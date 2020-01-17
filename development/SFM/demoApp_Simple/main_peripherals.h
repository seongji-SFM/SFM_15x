#ifndef __MAIN_PERIPHERALS_H__
#define __MAIN_PERIPHERALS_H__
#ifdef __cplusplus
extern "C" {
#endif
#define CFG_I2C_TIMEOUT 100000

int internal_Acc_CheckInit(void);
int internal_Acc_Shutdown(void);
bool internal_Acc_IsDetected(void);
uint16_t internal_Acc_Get_Sensitivity(void);
bool internal_Acc_Control_Open_Drain(bool bIsOpenDrain);
int internal_Acc_Get_XYZ(int16_t *x, int16_t *y, int16_t *z);


#ifdef __cplusplus
}
#endif

#endif // __CFG_EXTERNAL_PERIPHERALS_H__

