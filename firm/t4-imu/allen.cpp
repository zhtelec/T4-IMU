#define _ALLEN_CPP_
/*
allen分散の c code
function: AllenInit(uint32_t fc), AllenLoop(void), AllenStoreFreq(uint32_t freq)
AllenLoop(void) cpu resouce 分散計算はこちらでやる. AllenStoreFreq() を trigger にしない
AllenStoreFreq()  1 sec に 1回 data が登録される.
memory 100kB (102400bytes)
buffer は ring buffer,
data は順次入力  1sec 毎に入力する関数を用意 ring buffer に入れる.

tau を 1, 1.778, 3.162, 5.623, 10, 17.78, ... と 100kB で有効な tau をすべて計算
dataを Fc の差分 +-127 までの int8_t で記録
-128 は invalid で計算から抜く.
invalid data 個数が 10個以上でそれより上の tau は計算しない
*/

/**
 * allen_variance.c  -  Allan分散 (ADEV) 実装
 *
 * τ系列: 10^(0.25*k) を四捨五入した整数サンプル数 m[k]
 *        k=0〜18 の計19点 (102400サンプルで有効な全点)
 *
 * Allan分散の定義式:
 *
 *   σ²(τ) = 1/(2·K) · Σ_{k=0}^{K-1} ( y[k+2m] - 2·y[k+m] + y[k] )²
 *
 *   ただし K = N - 2m (有効トリプレット数, invalid を除く)
 *
 * CPU 分散戦略:
 *   AllenLoop() 1回で 1τ分の全ペアを走査して σ² を確定する。
 *   19τあるので、19回の呼び出しで全結果が更新される。
 *   その後 τ=0 に戻ってラウンドロビンを繰り返す。
 */

#include        <Arduino.h>
#include        <float.h>
#include        <stdint.h>
#include        <string.h>
#include        <math.h>

#include        "config.h"
#include        "allen.h"

/* ------------------------------------------------------------------ */
/*  内部定数                                                            */
/* ------------------------------------------------------------------ */

#define INVALID_SAMPLE   (FLT_MAX)         /**< 無効値マーカー           */
#define INVALID_LIMIT    10u               /**< これ以上で上位τをスキップ */

/*
 * τ系列の整数サンプル数テーブル
 * m[i] = round( 10^(0.25·i) ), i=0..18
 * 条件: 2·m + 1 <= ALLEN_RING_SIZE
 *
 *  i   τ[s]      m
 *  0     1.00      1
 *  1     1.78      2
 *  2     3.16      3
 *  3     5.62      6
 *  4    10.00     10
 *  5    17.78     18
 *  6    31.62     32
 *  7    56.23     56
 *  8   100.00    100
 *  9   177.83    178
 * 10   316.23    316
 * 11   562.34    562
 * 12  1000.00   1000
 * 13  1778.28   1778
 * 14  3162.28   3162
 * 15  5623.41   5623
 * 16 10000.00  10000
 * 17 17782.79  17783
 * 18 31622.78  31623
 */
static const uint32_t TAU_M[ALLEN_TAU_COUNT] = {
        1u,     2u,     3u,     6u,    10u,
       18u,    32u,    56u,   100u,   178u,
      316u,   562u,  1000u,  1778u,  3162u
      //5623u, 10000u, 17783u, 31623u
};

/* ------------------------------------------------------------------ */
/*  Ring buffer                                                         */
/* ------------------------------------------------------------------ */

DMAMEM static float     s_buf[ALLEN_RING_SIZE]; /**< 本体                   */
static uint32_t  s_write;               /**< 次回書き込み位置          */
static uint32_t  s_count;               /**< 格納済みサンプル数        */
static float     s_fc;                  /**< 基準周波数                */

static uint32_t  s_tMeasurement;

/* ------------------------------------------------------------------ */
/*  結果配列                                                         */
/* ------------------------------------------------------------------ */

static float allen_result[ALLEN_TAU_COUNT];
static float adev_result[ALLEN_TAU_COUNT];

/* ------------------------------------------------------------------ */
/*  内部ヘルパー                                                        */
/* ------------------------------------------------------------------ */

/**
 * ring_get()
 *
 * 相対インデックス rel (0=最古サンプル, s_count-1=最新) でサンプルを返す。
 * s_count > 0 かつ rel < s_count であることを呼び出し元が保証すること。
 */
static inline float ring_get(uint32_t rel)
{
    /*
     * 最古の絶対位置:
     *   s_write が次回書き込み位置なので、
     *   最古 = (s_write - s_count + RING_SIZE) % RING_SIZE
     */
    uint32_t abs_pos = (s_write + ALLEN_RING_SIZE - s_count + rel)
                       % ALLEN_RING_SIZE;
    return s_buf[abs_pos];
}

/**
 * compute_adev()
 *
 * τ = m サンプルの Allan分散を計算して返す。
 * 戻り値   : σ²(τ)  (有効ペア0のとき ALLEN_RESULT_INVALID)
 * *out_inv : 無効サンプルを含んでいたトリプレット数
 */
static float compute_adev(uint32_t m, uint32_t *out_inv)
{
    uint32_t N        = s_count;
    uint32_t inv_cnt  = 0u;
    uint32_t used     = 0u;
    float    sum      = 0.0f;

    *out_inv = 0u;

    /* データ最低要件: y[0], y[m], y[2m] の 3点が必要 */
    if (N < (2u * m + 1u)) {
        return ALLEN_RESULT_INVALID;
    }

    uint32_t K = N - 2u * m;   /* トリプレット総数 */

    for (uint32_t k = 0u; k < K; k++) {
        float y0  = ring_get(k);
        float ym  = ring_get(k + m);
        float y2m = ring_get(k + 2u * m);

        /* いずれかが invalid → スキップ、カウントのみ */
        if ((y0  == INVALID_SAMPLE) ||
            (ym  == INVALID_SAMPLE) ||
            (y2m == INVALID_SAMPLE))
        {
            inv_cnt++;
            continue;
        }

        /* ( y[k+2m] - 2·y[k+m] + y[k] )² を浮動小数で累積 */
        float d = (float)y2m - 2.0f * (float)ym + (float)y0;
        sum += d * d;
        used++;
    }

    *out_inv = inv_cnt;

    if (used == 0u) {
        return ALLEN_RESULT_INVALID;
    }

    /* σ²(τ) = sum / (2 · used) */
    return sum / (2.0f * (float)used);
}

/* ------------------------------------------------------------------ */
/*  公開 API                                                            */
/* ------------------------------------------------------------------ */

void AllenInit(uint32_t fc)
{
    s_fc      = (float)fc;
    s_write   = 0u;
    s_count   = 0u;
    //s_tau_idx = 0u;
    s_tMeasurement = 0;

    memset(&allen_result, 0, sizeof(allen_result));
    memset(&adev_result, 0, sizeof(adev_result));

    /* 全バッファを invalid で初期化 */
    for (uint32_t i = 0u; i < ALLEN_RING_SIZE; i++) {
        s_buf[i] = INVALID_SAMPLE;
    }

    /* 結果をリセット */
    for (uint32_t i = 0u; i < ALLEN_TAU_COUNT; i++) {
        allen_result[i] = ALLEN_RESULT_INVALID;
        adev_result[i]  = ALLEN_RESULT_INVALID;
    }
}

/* ------------------------------------------------------------------ */
void AllenLoop(void)
{
}

/* ------------------------------------------------------------------ */

void AllenCalc(void)
{
  /* 最低限 3サンプル必要 (τ=1 でも y0, y1, y2 の 3点) */
  if (s_count < 3u) {
    return;   /* データ不足: 次回呼び出しまで待つ */
  }

  uint32_t cntInvalid = 0u;

  for(int i = 0; i < ALLEN_TAU_COUNT; i++) {
    allen_result[i] = compute_adev(TAU_M[i], &cntInvalid);

    /* invalid 超過判定: このτより大きいτは信頼性なし */
    if(cntInvalid >= INVALID_LIMIT) {
      /* 残りのτを INVALID にしてサイクルをリセット */
      for (uint32_t j = i + 1u; j < ALLEN_TAU_COUNT; j++) {
        allen_result[j] = ALLEN_RESULT_INVALID;
        adev_result[j]  = ALLEN_RESULT_INVALID;
      }
      return;
    }

    adev_result[i] = sqrtf(allen_result[i]) / s_fc;
  }

  return;
}

/* ------------------------------------------------------------------ */

void AllenStoreFreq(float freq)
{
  float diff = freq;
  float sample;

  if ((diff < -127.0) || (diff > 127.0)) {
    sample = INVALID_SAMPLE;          /* 範囲外 → invalid */
  } else {
    sample = (float)diff;
  }

  /* ring buffer に書き込む */
  s_buf[s_write] = sample;
  s_write = (s_write + 1u) % ALLEN_RING_SIZE;

  if (s_count < ALLEN_RING_SIZE) {
    s_count++;
  }

  /*
   * s_count == ALLEN_RING_SIZE のとき満杯。
   * write が最古を上書きするので count はそのまま維持。
   */

  s_tMeasurement++;
}


int AllenGetResult(const uint32_t **pTime, float **pResult)
{
  *pResult = adev_result;
  *pTime = TAU_M;
  return ALLEN_TAU_COUNT;
}
uint32_t AllenGetMeasureTime(void)
{
  return s_tMeasurement;
}


void AllenFlush(int cnt)
{
  if(       cnt < 0) {
    s_count = 0;

  } else if(s_count > cnt) {
    s_count -= cnt;

  }

  return;
}

