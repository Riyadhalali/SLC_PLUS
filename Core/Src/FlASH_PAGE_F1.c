#include "FLASH_PAGE_F1.h"
#include "string.h"
#include "stdio.h"

/* STM32F103 have 128 PAGES (Page 0 to Page 127) of 1 KB each. This makes up 128 KB Flash Memory
 * Some STM32F103C8 have 64 KB FLASH Memory, so I guess they have Page 0 to Page 63 only.
 */

/* FLASH_PAGE_SIZE should be able to get the size of the Page according to the controller */
static uint32_t GetPage(uint32_t Address)
{
  for (int indx = 0; indx < 128; indx++)
  {
    if ((Address < (0x08000000 + (FLASH_PAGE_SIZE * (indx + 1)))) && (Address >= (0x08000000 + FLASH_PAGE_SIZE * indx)))
    {
      return (0x08000000 + FLASH_PAGE_SIZE * indx);
    }
  }

  return 0;
}

uint8_t bytes_temp[4];

void float2Bytes(uint8_t *ftoa_bytes_temp, float float_variable)
{
  union {
    float a;
    uint8_t bytes[4];
  } thing;

  thing.a = float_variable;

  for (uint8_t i = 0; i < 4; i++)
  {
    ftoa_bytes_temp[i] = thing.bytes[i];
  }
}

float Bytes2float(uint8_t *ftoa_bytes_temp)
{
  union {
    float a;
    uint8_t bytes[4];
  } thing;

  for (uint8_t i = 0; i < 4; i++)
  {
    thing.bytes[i] = ftoa_bytes_temp[i];
  }

  return thing.a;
}

uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords)
{
  static FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PAGEError;
  int sofar = 0;

  /* Disable interrupts during critical Flash operations */
  __disable_irq();

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area */
  uint32_t StartPage = GetPage(StartPageAddress);
  uint32_t EndPageAdress = StartPageAddress + numberofwords * 4;
  uint32_t EndPage = GetPage(EndPageAdress);

  /* Fill EraseInit structure */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = StartPage;
  EraseInitStruct.NbPages = ((EndPage - StartPage) / FLASH_PAGE_SIZE) + 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
  {
    __enable_irq(); // Re-enable interrupts in case of error
    return HAL_FLASH_GetError();
  }

  /* Program the user Flash area word by word */
  while (sofar < numberofwords)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartPageAddress, Data[sofar]) == HAL_OK)
    {
      StartPageAddress += 4; // increment address by word size
      sofar++;
    }
    else
    {
      __enable_irq(); // Re-enable interrupts in case of error
      return HAL_FLASH_GetError();
    }
  }

  /* Lock the Flash to disable the flash control register access */
  HAL_FLASH_Lock();

  /* Re-enable interrupts after Flash operation is complete */
  __enable_irq();

  return 0;
}

void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
  while (numberofwords--)
  {
    *RxBuf = *(__IO uint32_t *)StartPageAddress;
    StartPageAddress += 4;
    RxBuf++;
  }
}

void Convert_To_Str(uint32_t *Data, char *Buf)
{
  int numberofbytes = ((strlen((char *)Data) / 4) + ((strlen((char *)Data) % 4) != 0)) * 4;

  for (int i = 0; i < numberofbytes; i++)
  {
    Buf[i] = Data[i / 4] >> (8 * (i % 4));
  }
}

void Flash_Write_NUM(uint32_t StartSectorAddress, float Num)
{
  float2Bytes(bytes_temp, Num);
  Flash_Write_Data(StartSectorAddress, (uint32_t *)bytes_temp, 1);
}

float Flash_Read_NUM(uint32_t StartSectorAddress)
{
  uint8_t buffer[4];
  float value;

  Flash_Read_Data(StartSectorAddress, (uint32_t *)buffer, 1);
  value = Bytes2float(buffer);
  return value;
}
