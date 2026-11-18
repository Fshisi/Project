#ifndef SYSTEM_CLOCK_H
#define SYSTEM_CLOCK_H
/* HSI 8MHz / 2 * 16 = 标称64MHz；不依赖外部晶振。 */
void SystemClock_Config(void);
#endif
