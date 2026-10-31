/* ==========================================================================
 *  MicPowerBank -- ATtiny84A 麦克风自动充电仓主控固件
 * --------------------------------------------------------------------------
 *  开发环境 : Arduino IDE 1.8.x / 2.x  +  ATTinyCore (推荐) 或经典 ATtiny 核心
 *  目标芯片 : ATtiny84A   内部 RC 1MHz, BOD 建议烧录为 1.8V 或 Disabled
 *  编译器   : avr-gcc (Arduino)   F_CPU 用 1MHz 或 8MHz 均可(程序运行时会
 *             通过 CLKPR 把时钟强制分成 1MHz, 采样周期由看门狗振荡器决定)
 *
 *  ---------------------------------------------------------------- 引脚
 *    KEY  PB2 (pin 5)  按键输入(低有效): 使能内部上拉, 按下对地短路
 *    LED  PA5 (pin 8)  指示灯: LED 阳极 -> 限流电阻 -> VCC, 阴极 -> PA5
 *                      (高电平点亮; 若你的 LED 是接 VCC, 把 LED_ACTIVE_LOW 改 1)
 *    MOS  PA6 (pin 7)  N 沟道 MOS 栅极(充电输出控制)
 *    RST  PB3 (pin 4)  10k 上拉到 VCC, 建议并联 100nF 到 GND(抗干扰)
 *
 *  ------------------------------------------------------------ 硬件建议
 *    * VCC/GND 之间 100nF + 10uF 去耦, 尽量靠近芯片
 *    * MOS: 栅极串 100Ω, 栅-源之间 100k 下拉(复位期间保证关断)
 *          必须选"逻辑电平" MOS(如 AO3400/SI2302, 2.5V 下 Rds(on) 已足够低)
 *    * LED: 限流电阻 1k 左右(红色 LED 约 1~2mA), 用高亮低电流 LED 更省电
 *    * 未使用的引脚程序里已配置成"输入+上拉", 防止悬空漏电
 *
 *  ------------------------------------------------------------ 功能说明
 *    1. 上电即进入 充电前期(不输出); 前期计时到点后自动进入充电期,
 *       形成"前期 9h -> 充电 4h -> 后期 11h -> 前期 ..."的 24h 自动循环
 *       (PRE_AUTO_CHG=0 时前期不自动推进, 只能靠按键开始充电)
 *    2. 单击按键 -> 立即进入 充电期(MOS 导通), 此刻作为充电期起点计时
 *       充电期结束后停止输出, 去向由"进入方式"决定:
 *         · 自动进入的充电期(前期计时到点)走完 -> 正常进入 充电后期
 *         · 按键手动进入的充电期走完          -> 跳过充电后期, 直接回 充电前期
 *       充电期无论怎么进入, 只要被单击打断, 就一律回充电前期
 *    3. 充电后期计时到点 -> 充电前期; 后期内单击 -> 立即进入充电期(算手动进入,
 *       结束后回前期)
 *    4. 低电闪烁提醒期间长按 5s -> 停止本轮低电闪烁提醒;
 *       电池电压恢复到 VBAT_LOW_EXIT_MV 以上后, 再次低于阈值会重新提醒
 *    5. 长按 10s -> 指示灯快闪 3 下, 进入"时长调节模式", 详见下方调节说明
 *    6. 电池电压通过内部 1.1V 基准自测(无需外部电阻分压);
 *       低于 VBAT_LOW_ENTER_MV 后指示灯按档位闪烁, 电量越低越快:
 *       从 5s 一次逐档缩短到 300ms 一次(每档为固定频率, 不连续变化)
 *    7. 平时 MCU 处于 Power-Down, 由看门狗定时器(16ms~8.2s 自适应)唤醒,
 *       睡眠时关闭 BOD, 关闭 Timer0/1/USI/ADC, 静态电流约几 uA
 *    8. 单击判定: 只要松手时"还没按到第一个长按门限"就算单击(不留空档区);
 *       运行模式下单击被识别时指示灯"快闪 2 下"(CLICK_FEEDBACK=1),
 *       调节模式里单击是"闪 1 下", 数闪的次数即可知道自己处于哪个模式
 *    9. 调节模式 2 分钟无任何操作会自动放弃退出(闪 5 下), 防止误入后"按键只
 *       加小时、充电输出始终不动作"的假死现象
 *
 *  ------------------------------------------------------------ 计时精度
 *    Power-Down 下唯一可用时基是看门狗 RC 振荡器(128kHz), 数据手册给出
 *    ±10% 误差(常温常温下通常 ±2~5%), 因此 9h/4h/11h 会有分钟级偏差。
 *    若需要更高精度, 见文件末尾"改进建议"。
 *
 *  ------------------------------------------------------------ 调节说明
 *    进入 : 运行中长按按键 10s  -> 快闪 3 下 -> 进入调节模式,
 *           从第一项"充电前期"开始; 调节期间输出状态不变、阶段计时暂停
 *    加时长: 每单击一次 +0.5h 并闪 1 下
 *    下一项: 长按 5s -> 快闪 2 下 -> 依次是 前期 -> 充电期 -> 后期
 *    保存退出: 在第三项"充电后期"长按 5s -> 快闪 3 下 -> 写入 EEPROM
 *    放弃退出: 任意一项长按 15s -> 快闪 5 下 -> 不保存, 恢复原来的时长
 *    上限 : 单项最大 24h; 保存时 0h 会被钳成 0.5h
 *    单项规则: 进入调节时三项都先保留原来的时长;
 *           某一项如果一次都不单击, 就原样保留(长按 5s 跳到下一项即可);
 *           某一项只要单击过一次, 该项就从 0h 重新算, 点一下加 0.5h
 *           (所以想改小也不用先清零, 直接重数一遍就行)
 *    退出后阶段计时从退出时刻继续, 充电期剩余时间不会被调节过程吃掉
 * ==========================================================================*/

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

/* ======================= 一、可调参数 ======================= */

/* ---- 时间参数(ms) ---- */
#define KEY_SAMPLE_MS      16       /* 按键采样周期           */
#define KEY_DEBOUNCE_N     3        /* 连续 N 次采样一致才确认(≈48ms) */
#define ADJ_ENTER_MS       10000UL  /* 运行模式: 长按 10s 进入调节模式 */
#define ADJ_NEXT_MS        5000UL   /* 调节模式: 长按 5s 下一阶段/保存退出 */
#define ADJ_ABORT_MS       15000UL  /* 调节模式: 长按 15s 放弃退出 */
#define ADJ_IDLE_EXIT_MS   120000UL /* 调节模式无操作自动放弃退出(防止卡在调节模式) */
#define LOWBAT_MUTE_MS     5000UL   /* 低电提醒中: 长按 5s 停止本轮提醒 */
#define ADJ_MAX_HALF_HOURS 48       /* 单阶段最大 24h, 以 0.5h 为单位 */
#define HALF_HOUR_MS       1800000UL

/* ---- 行为开关 ---- */
#define PRE_AUTO_CHG       1        /* 1: 充电前期计时到点自动进入充电期(自动充电周期)
                                     *    该充电期结束后进充电后期
                                     * 0: 前期不自动推进, 只能按键手动开始充电,
                                     *    手动充电期结束后进充电前期          */
#define CLICK_FEEDBACK     0        /* 1: 运行模式下按一下(开始/停止充电)指示灯快闪 2 下,
                                     *    方便确认"按键确实被识别到了";
                                     *    调节模式里单击是"闪 1 下", 运行模式是"快闪 2 下",
                                     *    数闪的次数就知道自己处于哪个模式             */
#define SELFTEST_ON_BOOT   0        /* 1: 上电自检 —— 指示灯与 PA6(充电输出)各闪 3 次,
                                     *    用来核对引脚/接线是否正常(会短暂导通 MOS) */

/* ---- 指示灯闪烁参数 ---- */
#define PAT_ON_MS          80       /* "快闪"点亮时长 */
#define PAT_OFF_MS         120      /* "快闪"熄灭间隔 */
#define LOWBAT_ON_MS       60       /* 低电指示点亮时长(短, 省电) */

/* ---- 电池检测 ---- */
#define BATT_SAMPLE_MS     10000UL  /* 电压采样周期 */
#define VBAT_MUX           0x21     /* ADMUX MUX[5:0]=0b100001 -> 内部 1.1V 通道 */
#define VREF_MV            1100UL   /* 内部基准典型值 1.1V */
#define VBAT_CAL_PERMILLE  1000UL   /* 电压校准(千分比): 读数偏高 x% 则改成 (1000-x) */
#define VBAT_LOW_ENTER_MV  3350     /* 低于此值 -> 开始低电闪烁 */
#define VBAT_LOW_EXIT_MV   3450     /* 高于此值 -> 停止低电闪烁(迟滞) */
#define VBAT_MIN_VALID_MV  500      /* 小于此值认为是无效采样, 丢弃 */

/* ---- 阶段索引 ---- */
#define PH_PRE             0
#define PH_CHG             1
#define PH_POST            2

/* ---- 默认时长(0.5h): 前期 / 充电期 / 后期 ---- */
static const uint16_t kDefHalfHours[3] = {18, 8, 22};

/* ---- 低电闪烁档位: 电压越低于阈值, 档位越靠后, 周期越短 ---- */
#define BATT_LVLS 8
static const uint16_t kLvlMv[BATT_LVLS]       = {3350, 3300, 3250, 3200, 3150, 3100};
static const uint16_t kLvlPeriodMs[BATT_LVLS] = {10000, 5000, 3200, 1500, 1000,  700};

/* ---- 看门狗分频档位: 索引 0..9 对应 16ms..8.192s ---- */
#define WDT_LEVELS 10
static const uint16_t kWdtMs[WDT_LEVELS] = {16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192};

/* ---- EEPROM 配置 ---- */
#define CFG_MAGIC   0xA5
#define CFG_VER     2
#define CFG_VER_LEGACY_HOURS 1
typedef struct {
  uint8_t  magic;
  uint8_t  ver;
  uint16_t halfHours[3];  /* 前期 / 充电期 / 后期, 单位 0.5h */
  uint8_t  chk;
} cfg_t;

/* V1 配置用整小时; 读取后迁移成 0.5h 单位 */
typedef struct {
  uint8_t  magic;
  uint8_t  ver;
  uint16_t hours[3];
  uint8_t  chk;
} cfg_v1_t;

/* ======================= 二、引脚定义 ======================= */

/* KEY: PB2, 内部上拉, 按下为低 */
#define KEY_PINR      PINB
#define KEY_PORT      PORTB
#define KEY_DDR       DDRB
#define KEY_BMASK     (1 << PB2)
#define KEY_PCMSK     PCMSK1
#define KEY_PCINT_BIT 2                            /* PB2 = PCINT10 = PCMSK1 的 bit2 */
#define KEY_PCIE      PCIE1

/* LED: PA5, 低电平点亮(可改) */
#define LED_PORT      PORTA
#define LED_DDR       DDRA
#define LED_BMASK     (1 << PA5)
#define LED_ACTIVE_LOW 0

/* MOS: PA6, 高电平导通 */
#define MOS_PORT      PORTA
#define MOS_DDR       DDRA
#define MOS_BMASK     (1 << PA6)

/* ======================= 三、全局状态 ======================= */

static volatile uint32_t g_now;        /* 毫秒时基, 由看门狗中断累加 */
static volatile uint16_t g_step;       /* 当前看门狗周期(ms), 供中断使用 */

/* --- 电池 --- */
static uint16_t g_vbat;                /* 最近电压(mV) */
static uint8_t  g_vbatValid;
static uint8_t  g_battLow;             /* 低电标志 */
static uint8_t  g_lowBattMuted;        /* 本轮低电提醒已由长按静音 */
static uint8_t  g_battLvl;             /* 低电档位 */
static uint32_t g_nextAdc;             /* 下次采样时刻 */
static uint32_t g_lowNext;             /* 下次低电闪灯时刻 */

/* --- 指示灯图案播放 --- */
static uint8_t  g_patLeft;             /* 剩余点亮次数 */
static uint8_t  g_patOn;               /* 当前是否点亮 */
static uint16_t g_patOnMs, g_patOffMs;
static uint32_t g_patNext;

/* --- 按键 --- */
static uint8_t  g_keyRaw, g_keyRawCnt, g_keyDown;
static uint32_t g_keySampleNext, g_keyDownMs;
static uint8_t  g_holdLock, g_hold5, g_hold15;

/* --- 主状态机 --- */
static uint8_t  g_mode;                /* 0=运行, 1=时长调节 */
static uint8_t  g_phase;               /* PH_PRE / PH_CHG / PH_POST */
static uint8_t  g_chgAuto;             /* 当前充电期是否为"自动进入"(决定结束后去向) */
static uint32_t g_phaseStart;
static uint16_t g_durHalfHours[3];     /* 生效中的时长(0.5h) */
static uint32_t g_durMs[3];            /* 生效中的时长(ms) */

/* --- 调节模式临时数据 --- */
static uint8_t  g_adjStage;            /* 0=前期 1=充电期 2=后期 */
static uint16_t g_adjHalfHours[3];     /* 调节中的临时值(未单击过的项保持原值) */
static uint8_t  g_adjTouched[3];       /* 该项本次调节是否单击过 */
static uint32_t g_adjEnterMs;
static uint32_t g_adjLastAct;          /* 调节模式最后一次操作时刻(用于超时退出) */

/* ======================= 四、基础 IO ======================= */

void ledWrite(uint8_t on)
{
#if LED_ACTIVE_LOW
  if (on) LED_PORT &= (uint8_t)~LED_BMASK;
  else    LED_PORT |= LED_BMASK;
#else
  if (on) LED_PORT |= LED_BMASK;
  else    LED_PORT &= (uint8_t)~LED_BMASK;
#endif
}

void mosWrite(uint8_t on)
{
  if (on) MOS_PORT |= MOS_BMASK;
  else    MOS_PORT &= (uint8_t)~MOS_BMASK;
}

/* 粗略毫秒延时(只给上电自检用)。setup 里时钟已分成 1MHz, 空循环约 4~5 周期,
 * 所以 200 次≈1ms, 精度无所谓。                                    */
void selfTestDelayMs(uint16_t ms)
{
  while (ms--) {
    volatile uint8_t n = 200;
    while (n--) { }
  }
}

uint8_t keyIsDownRaw(void)
{
  return (KEY_PINR & KEY_BMASK) ? 0 : 1;   /* 1 = 按下 */
}

/* ======================= 五、EEPROM 配置存取 ======================= */

uint8_t cfgChkRaw(const uint8_t *p, uint8_t len)
{
  uint8_t s = 0;
  uint8_t i;
  for (i = 0; i < (uint8_t)(len - 1); i++) s = (uint8_t)(s + p[i]);
  return (uint8_t)(s ^ 0x5A);
}

uint8_t cfgChk(const cfg_t *c)
{
  return cfgChkRaw((const uint8_t *)c, sizeof(cfg_t));
}

uint8_t cfgV1Chk(const cfg_v1_t *c)
{
  return cfgChkRaw((const uint8_t *)c, sizeof(cfg_v1_t));
}

uint16_t cfgClamp(uint16_t halfHours)
{
  if (halfHours < 1) halfHours = 1;    /* 0 无意义, 钳到 0.5h */
  if (halfHours > ADJ_MAX_HALF_HOURS) halfHours = ADJ_MAX_HALF_HOURS;
  return halfHours;
}

void cfgLoad(void)
{
  cfg_t c;
  uint8_t i;

  eeprom_read_block((void *)&c, (const void *)0, sizeof(c));
  {
    uint8_t current = (c.magic == CFG_MAGIC) &&
                      (c.ver == CFG_VER) &&
                      (c.chk == cfgChk(&c));
    cfg_v1_t old;
    uint8_t legacy = 0;

    if (!current) {
      eeprom_read_block((void *)&old, (const void *)0, sizeof(old));
      legacy = (old.magic == CFG_MAGIC) &&
               (old.ver == CFG_VER_LEGACY_HOURS) &&
               (old.chk == cfgV1Chk(&old));
    }

    for (i = 0; i < 3; i++) {
      uint16_t halfHours = current ? c.halfHours[i] :
                           legacy  ? (uint16_t)(old.hours[i] * 2) :
                                     kDefHalfHours[i];
      halfHours = cfgClamp(halfHours);
      g_durHalfHours[i] = halfHours;
      g_durMs[i]        = (uint32_t)halfHours * HALF_HOUR_MS;
    }
  }
}

void cfgSave(void)
{
  cfg_t c;
  uint8_t i;
  c.magic = CFG_MAGIC;
  c.ver   = CFG_VER;
  for (i = 0; i < 3; i++) c.halfHours[i] = g_durHalfHours[i];
  c.chk = cfgChk(&c);
  eeprom_update_block((const void *)&c, (void *)0, sizeof(c));
}

/* ======================= 六、指示灯图案 ======================= */

uint8_t patBusy(void) { return (uint8_t)(g_patOn || g_patLeft); }

void patStart(uint8_t flashes, uint16_t onMs, uint16_t offMs)
{
  g_patLeft  = flashes;
  g_patOn    = 0;
  g_patOnMs  = onMs;
  g_patOffMs = offMs;
  g_patNext  = g_now;                  /* 立即开始 */
  if (flashes == 0) ledWrite(0);
}

void patProcess(uint32_t now)
{
  if (!patBusy()) return;
  if ((int32_t)(now - g_patNext) < 0) return;

  if (g_patOn) {                       /* 点亮结束 -> 熄灭 */
    ledWrite(0);
    g_patOn = 0;
    if (g_patLeft) g_patLeft--;
    g_patNext = now + g_patOffMs;
  } else if (g_patLeft) {              /* 间隔结束 -> 点亮 */
    ledWrite(1);
    g_patOn = 1;
    g_patNext = now + g_patOnMs;
  }
}

/* ======================= 七、电池电压检测 ======================= */

/* 用内部 1.1V 基准反推 VCC(即 18650 电压), 不需要外部电阻分压。
 * 通道选择: ATtiny84A MUX[5:0]=0b100001(0x21) 为内部 1.1V。
 * 第一次转换丢弃(基准建立时间), 之后取 4 次平均。               */
uint16_t adcReadVccMv(void)
{
  uint16_t acc = 0;
  uint8_t  i;

#ifdef PRADC
  PRR &= (uint8_t)~_BV(PRADC);                       /* 打开 ADC 电源 */
#endif
  ADMUX  = (0 << REFS1) | (0 << REFS0) | VBAT_MUX;   /* 参考 = VCC */
  ADCSRB = 0;
  ADCSRA = (1 << ADEN) | (1 << ADPS2);               /* 使能, 时钟/16 -> 62.5kHz */

  ADCSRA |= (1 << ADSC);                             /* 丢弃的第一次 */
  i = 250;
  while ((ADCSRA & (1 << ADSC)) && i) i--;           /* 带超时, 防死等 */

  for (i = 0; i < 4; i++) {
    uint8_t lo;
    uint8_t t = 250;
    ADCSRA |= (1 << ADSC);
    while ((ADCSRA & (1 << ADSC)) && t) t--;
    lo = ADCL;                                       /* 先读低字节, 锁存高字节 */
    acc += (uint16_t)(lo | ((uint16_t)ADCH << 8));
  }
  ADCSRA = 0;                                        /* 关闭 ADC 省电 */
#ifdef PRADC
  PRR |= (1 << PRADC);
#endif

  {
    uint16_t raw = (uint16_t)(acc >> 2);
    uint32_t mv;
    if (raw == 0) return 0;
    mv = (VREF_MV * 1024UL) / raw;                   /* 单位 mV */
    mv = (mv * VBAT_CAL_PERMILLE) / 1000UL;
    if (mv > 60000UL) mv = 60000UL;
    return (uint16_t)mv;
  }
}

uint8_t battLvlOf(uint16_t mv)
{
  uint8_t i;
  for (i = 0; i < BATT_LVLS; i++) {
    if (mv > kLvlMv[i]) return (i == 0) ? 0 : (uint8_t)(i - 1);
  }
  return (uint8_t)(BATT_LVLS - 1);
}

void battProcess(uint32_t now)
{
  uint16_t mv;

  if ((int32_t)(now - g_nextAdc) < 0) return;
  g_nextAdc = now + BATT_SAMPLE_MS;

  mv = adcReadVccMv();
  if (mv < VBAT_MIN_VALID_MV) return;                /* 无效值丢弃 */

  if (!g_vbatValid) { g_vbat = mv; g_vbatValid = 1; }
  else              { g_vbat = (uint16_t)(((uint32_t)g_vbat * 3 + mv) / 4); }  /* 轻滤波 */

  if (!g_battLow) {
    if (g_vbat < VBAT_LOW_ENTER_MV) { g_battLow = 1; g_battLvl = battLvlOf(g_vbat); g_lowNext = now; }
  } else if (g_vbat >= VBAT_LOW_EXIT_MV) {
    g_battLow = 0;
    g_lowBattMuted = 0;
  }

  if (g_battLow) {
    uint8_t lv = battLvlOf(g_vbat);
    if (lv != g_battLvl) { g_battLvl = lv; g_lowNext = now; }   /* 频率换档立即生效 */
  }
}

/* 低电闪灯: 每个周期点亮一次(波形尽量窄, 省电) */
void lowBattProcess(uint32_t now)
{
  if (!g_battLow || g_lowBattMuted || patBusy()) return;
  if ((int32_t)(now - g_lowNext) < 0) return;

  patStart(1, LOWBAT_ON_MS, LOWBAT_ON_MS);
  g_lowNext = now + kLvlPeriodMs[g_battLvl];
}

/* ======================= 八、充放电状态机 ======================= */

/* autoMode: 1 = 由前期计时到点自动进入充电期, 0 = 按键手动进入 */
void phaseEnter(uint8_t ph, uint32_t now, uint8_t autoMode)
{
  g_phase      = ph;
  g_phaseStart = now;
  mosWrite(ph == PH_CHG ? 1 : 0);
  if (ph == PH_CHG) g_chgAuto = autoMode;
}

void fsmProcess(uint32_t now)
{
  if (g_mode != 0) return;                           /* 调节模式下冻结计时 */

  if (g_phase == PH_PRE) {
#if PRE_AUTO_CHG
    if ((int32_t)(now - (g_phaseStart + g_durMs[PH_PRE])) >= 0)
      phaseEnter(PH_CHG, now, 1);                    /* 前期到点 -> 自动充电 */
#endif
  } else if (g_phase == PH_CHG) {
    if ((int32_t)(now - (g_phaseStart + g_durMs[PH_CHG])) >= 0) {
      if (g_chgAuto) phaseEnter(PH_POST, now, 0);    /* 自动进入的 -> 正常进后期 */
      else           phaseEnter(PH_PRE,  now, 0);    /* 手动进入的 -> 跳过后期回前期 */
    }
  } else if (g_phase == PH_POST) {
    if ((int32_t)(now - (g_phaseStart + g_durMs[PH_POST])) >= 0)
      phaseEnter(PH_PRE, now, 0);                    /* 后期结束 -> 前期 */
  }
}

/* ======================= 九、调节模式 ======================= */

void adjEnter(uint32_t now)
{
  uint8_t i;
  g_mode      = 1;
  g_adjStage  = 0;
  for (i = 0; i < 3; i++) {
    g_adjHalfHours[i] = g_durHalfHours[i];           /* 先保留原值 */
    g_adjTouched[i] = 0;                             /* 该项还没被单击过 */
  }
  g_adjEnterMs  = now;
  g_adjLastAct  = now;
  patStart(3, PAT_ON_MS, PAT_OFF_MS);                /* 快闪 3 下 */
}

/* 调节结束: 期间冻结的阶段计时补回来 */
void adjExit(uint32_t now)
{
  g_phaseStart += (now - g_adjEnterMs);
  g_mode = 0;
}

void adjSaveExit(uint32_t now)
{
  uint8_t i;
  for (i = 0; i < 3; i++) {
    g_durHalfHours[i] = cfgClamp(g_adjHalfHours[i]);
    g_durMs[i]        = (uint32_t)g_durHalfHours[i] * HALF_HOUR_MS;
  }
  cfgSave();
  adjExit(now);
  g_holdLock = 1;                                    /* 本次按住不再触发 10s 动作 */
  patStart(3, PAT_ON_MS, PAT_OFF_MS);                /* 快闪 3 下(保存退出) */
}

void adjAbort(uint32_t now)
{
  adjExit(now);                                      /* 直接丢弃临时值 */
  patStart(5, PAT_ON_MS, PAT_OFF_MS);                /* 快闪 5 下(放弃退出, 区别于保存) */
}

void adjNextStage(uint32_t now)
{
  g_adjLastAct = now;
  if (g_adjStage < 2) {
    g_adjStage++;
    patStart(2, PAT_ON_MS, PAT_OFF_MS);              /* 快闪 2 下 -> 下一阶段 */
  } else {
    adjSaveExit(now);
  }
}

/* 调节模式长时间无操作 -> 自动放弃退出, 避免"卡在调节模式里, 按键只加小时、
 * 充电输出一直不动作"的假死现象 */
void adjIdleCheck(uint32_t now)
{
  if (g_mode == 0) return;
  if ((int32_t)(now - (g_adjLastAct + ADJ_IDLE_EXIT_MS)) >= 0) adjAbort(now);
}

/* ======================= 十、按键处理 ======================= */

void onClick(uint32_t now)
{
  if (g_mode != 0) {                                 /* 调节模式: +0.5h */
    uint8_t st = g_adjStage;
    g_adjLastAct = now;
    if (!g_adjTouched[st]) {                         /* 本项第一次单击 -> 从 0h 重算 */
      g_adjTouched[st] = 1;
      g_adjHalfHours[st] = 0;
    }
    if (g_adjHalfHours[st] < ADJ_MAX_HALF_HOURS) g_adjHalfHours[st]++;
    patStart(1, PAT_ON_MS, PAT_OFF_MS);              /* 闪 1 下 */
    return;
  }

  switch (g_phase) {
    case PH_PRE:  phaseEnter(PH_CHG, now, 0); break;  /* 手动开始充电 */
    case PH_POST: phaseEnter(PH_CHG, now, 0); break;  /* 后期内单击也手动开始充电 */
    case PH_CHG:  phaseEnter(PH_PRE, now, 0); break;  /* 单击打断: 一律回充电前期 */
    default:      break;
  }
#if CLICK_FEEDBACK
  /* 单击已被识别: 快闪 2 下(调节模式里单击是闪 1 下, 用次数就能区分模式) */
  patStart(2, PAT_ON_MS / 2, PAT_OFF_MS / 2);
#endif
}

void keyScan(uint32_t now)
{
  uint8_t raw;

  if ((int32_t)(now - g_keySampleNext) < 0) return;
  g_keySampleNext = now + KEY_SAMPLE_MS;

  raw = keyIsDownRaw();
  if (raw != g_keyRaw) { g_keyRaw = raw; g_keyRawCnt = 1; }
  else if (g_keyRawCnt < KEY_DEBOUNCE_N) g_keyRawCnt++;

  if (g_keyRawCnt >= KEY_DEBOUNCE_N && raw != g_keyDown) {
    g_keyDown = raw;
    if (g_keyDown) {                                 /* ---- 按下 ---- */
      g_keyDownMs = now;
      g_holdLock = 0;
      g_hold5 = 0;
      g_hold15 = 0;
    } else {                                         /* ---- 抬起 ---- */
      uint32_t held = now - g_keyDownMs;
      uint32_t limit = (g_mode != 0) ? ADJ_NEXT_MS : ADJ_ENTER_MS;
      uint8_t holdAction = g_holdLock;
      g_holdLock = 0;
      /* 只要没按到"第一个长按门限"就按单击处理, 中间不留空档区 */
      if (held < limit && !holdAction) onClick(now);
    }
  }
}

void keyHold(uint32_t now)
{
  uint32_t held;

  if (!g_keyDown || g_holdLock) return;
  held = now - g_keyDownMs;

  if (g_mode != 0) {                                 /* 调节模式 */
    if (!g_hold5 && held >= ADJ_NEXT_MS) {
      g_hold5 = 1;
      adjNextStage(now);
    }
    if (!g_hold15 && g_mode != 0 && held >= ADJ_ABORT_MS) {
      g_hold15 = 1;
      g_holdLock = 1;
      adjAbort(now);
    }
  } else {                                           /* 运行模式 */
    if (g_battLow && !g_lowBattMuted && !g_hold5 && held >= LOWBAT_MUTE_MS) {
      g_hold5 = 1;
      g_holdLock = 1;
      g_lowBattMuted = 1;                            /* 停止本轮低电提醒 */
      patStart(0, 0, 0);                             /* 立即结束正在播放的脉冲 */
    } else if (held >= ADJ_ENTER_MS) {
      g_holdLock = 1;
      adjEnter(now);
    }
  }
}

/* ======================= 十一、睡眠调度 ======================= */

uint32_t untilMs(uint32_t deadline, uint32_t now)
{
  int32_t d = (int32_t)(deadline - now);
  return (d <= 0) ? 0UL : (uint32_t)d;
}

/* 按键是否处于"需要高频采样"的状态:
 *   · 已确认按下        -> 要判长按、要等释放
 *   · 原始电平与已确认状态不一致 -> 正在消抖, 必须继续快速采样才能确认按下/抬起
 * 少了第二种情况, 消抖期间就会按"下一次外部事件"(最多 8.2s)去睡,
 * 导致按下要十几秒才被确认 -> 单击永远超时、长按严重延后。            */
uint8_t keyBusy(void)
{
  return (uint8_t)(g_keyDown || (g_keyRaw != g_keyDown));
}

/* 返回距离"下一次需要处理的事情"还有多少 ms */
uint32_t nextEventDelay(uint32_t now)
{
  uint32_t d = untilMs(g_nextAdc, now);
  uint32_t t;

  if (keyBusy()) {                                   /* 消抖中或已按下: 16ms 采样 */
    t = untilMs(g_keySampleNext, now);
    if (t < d) d = t;
  }

  if (g_keyDown && !g_holdLock) {                    /* 长按判定 */
    uint32_t thr = (g_mode != 0) ? (g_hold5 ? ADJ_ABORT_MS : ADJ_NEXT_MS) : ADJ_ENTER_MS;
    t = untilMs(g_keyDownMs + thr, now);
    if (t < d) d = t;
  }

  if (g_mode != 0) {                                 /* 调节模式超时自动退出 */
    t = untilMs(g_adjLastAct + ADJ_IDLE_EXIT_MS, now);
    if (t < d) d = t;
  }

  if (patBusy()) {
    t = untilMs(g_patNext, now);
    if (t < d) d = t;
  }

  if (g_battLow && !g_lowBattMuted && !patBusy()) {
    t = untilMs(g_lowNext, now);
    if (t < d) d = t;
  }

  if (g_mode == 0) {
    if (g_phase == PH_PRE) {
#if PRE_AUTO_CHG
      t = untilMs(g_phaseStart + g_durMs[PH_PRE], now);   /* 前期到点自动充电 */
#else
      t = 0xFFFFFFFFUL;                                  /* 前期不自动推进 */
#endif
    } else if (g_phase == PH_CHG) {
      t = untilMs(g_phaseStart + g_durMs[PH_CHG], now);
    } else {
      t = untilMs(g_phaseStart + g_durMs[PH_POST], now);
    }
    if (t < d) d = t;
  }

  return d;
}

/* 选择不超过 delay 的最大看门狗周期, 尽量少唤醒 */
uint8_t pickWdtLevel(uint32_t delay)
{
  int8_t i;
  for (i = (WDT_LEVELS - 1); i > 0; i--) {
    if ((uint32_t)kWdtMs[i] <= delay) return (uint8_t)i;
  }
  return 0;
}

void wdtSetLevel(uint8_t lvl)
{
  uint8_t wdp = (uint8_t)(lvl & 0x07);
  uint8_t sreg = SREG;

  if (lvl & 0x08) wdp |= (1 << 5);                   /* WDP3 在 WDTCSR bit5 */
  cli();
  wdt_reset();
  WDTCSR |= (1 << WDCE) | (1 << WDE);                /* 定时序列: 允许修改 */
  WDTCSR  = (uint8_t)((1 << WDIE) | wdp);            /* 只中断, 不复位 */
  SREG = sreg;
}

/* ======================= 十二、中断 ======================= */

/* 看门狗: 唯一的时基来源; 进入 ISR 后 WDIE 被硬件清零, 需要重新置位 */
ISR(WDT_vect)
{
  WDTCSR |= (1 << WDIE);
  g_now += g_step;
}

/* 按键引脚变化中断: 只用于把 CPU 从 Power-Down 唤醒, 具体处理在主循环 */
ISR(PCINT1_vect)
{
  GIFR = (1 << PCIF1);
}

/* ======================= 十三、初始化 / 主循环 ======================= */

void setup(void)
{
  /* 1. 上电先关看门狗(防止复位后仍以最短周期运行导致反复复位) */
  wdt_disable();

  cli();

  /* 2. 时钟: 内部 8MHz / 8 = 1MHz, 降低运行电流 */
  CLKPR = (1 << CLKPCE);
  CLKPR = 0x03;

  /* 3. 关闭用不到的外设, 进一步省电 */
  TIMSK0 = 0;
  TIMSK1 = 0;
  PRR = 0;
#ifdef PRTIM0
  PRR |= (1 << PRTIM0);
#endif
#ifdef PRTIM1
  PRR |= (1 << PRTIM1);
#endif
#ifdef PRUSI
  PRR |= (1 << PRUSI);
#endif
#ifdef PRADC
  PRR |= (1 << PRADC);
#endif

  /* 4. 引脚 */
  KEY_DDR  &= (uint8_t)~KEY_BMASK;                   /* 按键: 输入 */
  KEY_PORT |= KEY_BMASK;                             /*       内部上拉 */
  MOS_DDR  |= MOS_BMASK;                             /* MOS:  输出 */
  mosWrite(0);
  LED_DDR  |= LED_BMASK;                             /* LED:  输出 */
  ledWrite(0);

  /* 未使用引脚: 输入 + 上拉, 防止悬空漏电 */
  DDRA  &= (uint8_t)~((1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3) |
                      (1 << PA4) | (1 << PA7));
  PORTA |= (1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3) |
           (1 << PA4) | (1 << PA7);
  DDRB  &= (uint8_t)~((1 << PB0) | (1 << PB1));
  PORTB |= (1 << PB0) | (1 << PB1);

#if SELFTEST_ON_BOOT
  /* ---- 上电自检: 指示灯 + 充电输出(PA6) 各脉冲 3 次 ----
   * 用万用表或 LED 核对 PA6/LED 的接线与 MOS 是否正常。
   * 自检会短暂导通充电输出, 正式使用请把 SELFTEST_ON_BOOT 设回 0。 */
  for (uint8_t t = 0; t < 3; t++) {
    ledWrite(1); mosWrite(1); selfTestDelayMs(300);
    ledWrite(0); mosWrite(0); selfTestDelayMs(300);
  }
#endif

  /* 5. 读配置 + 初始状态: 上电进入充电前期 */
  cfgLoad();
  g_vbatValid = 0;
  g_vbat      = 0;
  g_battLow   = 0;
  g_lowBattMuted = 0;
  g_battLvl   = 0;
  g_mode      = 0;
  g_keyRaw    = keyIsDownRaw();
  g_keyRawCnt = KEY_DEBOUNCE_N;
  g_keyDown   = 0;
  g_holdLock  = 0;
  g_hold5 = 0;
  g_hold15 = 0;
  g_patLeft = 0;
  g_patOn   = 0;
  g_chgAuto  = 0;
  g_now       = 0;
  g_step      = kWdtMs[0];
  g_nextAdc   = 500;                                 /* 开机 0.5s 后首次采样 */
  g_lowNext   = 0;
  phaseEnter(PH_PRE, 0, 0);

  /* 6. 按键引脚变化中断(可把 MCU 从 Power-Down 唤醒) */
  KEY_PCMSK |= (uint8_t)(1 << KEY_PCINT_BIT);
  GIFR      = (uint8_t)(1 << PCIF1);
  GIMSK    |= (uint8_t)(1 << KEY_PCIE);

  sei();
}

void loop(void)
{
  uint32_t now;
  uint32_t delay;
  uint8_t  lvl;

  cli();
  now = g_now;

  /* 引脚一旦变化, 立刻重新采样按键(此刻 CPU 可能就是被它唤醒的) */
  if (keyIsDownRaw() != g_keyRaw) g_keySampleNext = now;

  keyScan(now);
  keyHold(now);
  adjIdleCheck(now);
  fsmProcess(now);
  battProcess(now);
  lowBattProcess(now);
  patProcess(now);

  /* 计算睡眠时长并选看门狗档位 */
  delay = nextEventDelay(now);
  lvl   = pickWdtLevel(delay);
  g_step = kWdtMs[lvl];
  wdtSetLevel(lvl);

  /* Power-Down, 同时关闭 BOD(BOD 本身约 20uA, 必须关) */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
#if defined(BODS) && defined(BODSE)
  sleep_bod_disable();
#endif
  sei();                                             /* 允许中断唤醒 */

  /* 万一按键电平刚好在上面的处理期间变化(引脚变化中断可能在下一次睡眠后才被
   * 处理), 这里再确认一次: 不睡, 直接回到 loop 顶部重新处理按键。
   * 此时看门狗会在下一次循环里被重新配置, 不会残留超时。            */
  if (keyIsDownRaw() != g_keyRaw) {
    sleep_disable();
    return;
  }

  sleep_cpu();
  sleep_disable();
}
