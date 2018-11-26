#ifndef IKSECURITY_DIGITAL_KEY_H
#define IKSECURITY_DIGITAL_KEY_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**@brief   Event IDs. */
typedef enum
{
      DK_BLE_INVAIL = 0, 
      DK_SYS_INIT = 1,
      DK_BLE_INIT = 2, 
      DK_BLE_FLASH_READY,
      DK_BLE_ADV_FAST, //Advertising event.
      DK_BLE_ADV_IDLE, //Advertising event.
      DK_BLE_CONNECTED,  
      DK_BLE_DISCONNECTED,
      DK_BLE_GATTS_TIMEOUT,
      DK_BLE_GATTC_TIMEOUT
} ik_ble_status_t;

/**@brief Enum Ble Profile Character type .
 */
 
/**@brief       Function for Logical processing of Ble Character values.
 *
 * @param[in] secChar        Ble Character identifier.
 * @param[in] inCharValue    Ble Character value.
 * @param[in] inCharLen      Ble Character value length.
 * @retval :: true if Successful, false otherwise.
 */

extern bool ikSecurityDKHandleCharacter(uint16_t secChar, uint8_t *inCharValue,uint32_t inCharLen);


/**@brief       Function for Logical processing of running time status .
 *
 * @param[in] secChar        Ble Character identifier.
 * @param[in] bleStatus      .
 * @retval :: true if Successful, false otherwise.
 */
extern bool ikSecuritySetRunningStatus(uint16_t bleStatus);

/**@brief       Function for Get SE status .
 *
 * @retval :: true if se authorized, false otherwise.
 */

extern bool ikSecurityIsAuthorized();

/**@brief       Function for Set PS/PE status .
 * @param[in] bleMac     if bleMac is null, it will save to default value.
 * @param[in] psValue    
 * @param[in] peValue    
 * @retval :: true if success, false otherwise.
 */
extern bool ikSetPSCalibration(const unsigned char * bleMac,unsigned char psvalue,unsigned char pevalue);
/**@brief       Function for get PS/PE status .
 * @param[in] bleMac  if bleMac is null, it will get to default value.
 * @param[in] psValue     
 * @param[in] peValue    
 * @retval :: true if success, false otherwise.
 */
extern bool ikGetPSCalibration(const unsigned char * bleMac,unsigned char *psvalue,unsigned char *pevalue);


#ifdef __cplusplus
}
#endif

#endif
