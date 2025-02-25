/**
  ******************************************************************************
  * @file    stm32l0xx_hal_pcd.c
  * @author  MCD Application Team
  * @brief   PCD HAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of the USB Peripheral Controller:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions 
  *           + Peripheral State functions
  *         
  @verbatim
  ==============================================================================
                    ##### How to use this driver #####
  ==============================================================================
    [..]
      The PCD HAL driver can be used as follows:

     (#) Declare a PCD_HandleTypeDef handle structure, for example:
         PCD_HandleTypeDef  hpcd;
        
     (#) Fill parameters of Init structure in HCD handle
  
     (#) Call HAL_PCD_Init() API to initialize the HCD peripheral (Core, Device core, ...) 

     (#) Initialize the PCD low level resources through the HAL_PCD_MspInit() API:
         (##) Enable the PCD/USB Low Level interface clock using 
              (+++) __HAL_RCC_USB_CLK_ENABLE();
           
         (##) Initialize the related GPIO clocks
         (##) Configure PCD pin-out
         (##) Configure PCD NVIC interrupt
    
     (#)Associate the Upper USB device stack to the HAL PCD Driver:
         (##) hpcd.pData = pdev;

     (#)Enable HCD transmission and reception:
         (##) HAL_PCD_Start();

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

#if !defined(STM32L011xx) && !defined(STM32L021xx) && !defined (STM32L031xx) && !defined (STM32L041xx) && !defined (STM32L051xx) && !defined (STM32L061xx) && !defined (STM32L071xx) && !defined (STM32L081xx)
#ifdef HAL_PCD_MODULE_ENABLED
/** @addtogroup STM32L0xx_HAL_Driver
  * @{
  */

/** @addtogroup PCD
  * @brief PCD HAL module driver
  * @{
  */

/** @addtogroup PCD_Private
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BTABLE_ADDRESS                  (0x000U)  
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd);
void PCD_WritePMA(USB_TypeDef  *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);
void PCD_ReadPMA(USB_TypeDef  *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);

/**
  * @}
  */
/* Private functions ---------------------------------------------------------*/


/** @addtogroup PCD_Exported_Functions
  * @{
  */

/** @addtogroup PCD_Exported_Functions_Group1
 *  @brief    Initialization and Configuration functions 
 *
@verbatim    
 ===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
 
@endverbatim
  * @{
  */

/**
  * @brief  Initializes the PCD according to the specified
  *         parameters in the PCD_InitTypeDef and create the associated handle.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd)
{ 
  uint32_t i = 0U;

  uint32_t wInterrupt_Mask = 0U;
  
  /* Check the PCD handle allocation */
  if(hpcd == NULL)
  {
    return HAL_ERROR;
  }
  
  /* Check the parameters */
  assert_param(IS_PCD_ALL_INSTANCE(hpcd->Instance));

  if(hpcd->State == HAL_PCD_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hpcd->Lock = HAL_UNLOCKED;

    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    HAL_PCD_MspInit(hpcd);
  }

  hpcd->State = HAL_PCD_STATE_BUSY;
  
 /* Init endpoints structures */
 for (i = 0U; i < hpcd->Init.dev_endpoints ; i++)
 {
   /* Init ep structure */
   hpcd->IN_ep[i].is_in = 1U;
   hpcd->IN_ep[i].num = i;
   /* Control until ep is actvated */
   hpcd->IN_ep[i].type = PCD_EP_TYPE_CTRL;
   hpcd->IN_ep[i].maxpacket =  0U;
   hpcd->IN_ep[i].xfer_buff = 0U;
   hpcd->IN_ep[i].xfer_len = 0U;
 }
 
 for (i = 0U; i < hpcd->Init.dev_endpoints ; i++)
 {
   hpcd->OUT_ep[i].is_in = 0U;
   hpcd->OUT_ep[i].num = i;
   /* Control until ep is activated */
   hpcd->OUT_ep[i].type = PCD_EP_TYPE_CTRL;
   hpcd->OUT_ep[i].maxpacket = 0U;
   hpcd->OUT_ep[i].xfer_buff = 0U;
   hpcd->OUT_ep[i].xfer_len = 0U;
 }
  
 /* Init Device */
 /*CNTR_FRES = 1*/
 hpcd->Instance->CNTR = USB_CNTR_FRES;
 
 /*CNTR_FRES = 0*/
 hpcd->Instance->CNTR = 0U;
 
 /*Clear pending interrupts*/
 hpcd->Instance->ISTR = 0U;
 
  /*Set Btable Adress*/
 hpcd->Instance->BTABLE = BTABLE_ADDRESS;
 
 /*set wInterrupt_Mask global variable*/
 wInterrupt_Mask = USB_CNTR_CTRM  | USB_CNTR_WKUPM | USB_CNTR_SUSPM /* | USB_CNTR_ERRM */ \
   | USB_CNTR_SOFM | USB_CNTR_ESOFM | USB_CNTR_RESETM;
 
  /*Set interrupt mask*/
 hpcd->Instance->CNTR = wInterrupt_Mask;
 
 hpcd->USB_Address = 0U;
 hpcd->State= HAL_PCD_STATE_READY;
  
 /* Activate LPM */
 if (hpcd->Init.lpm_enable ==1)
 {
   HAL_PCDEx_ActivateLPM(hpcd);
 }  
 /* Activate Battery charging */
 if (hpcd->Init.battery_charging_enable ==1)
 {
   HAL_PCDEx_ActivateBCD(hpcd);
  }
 
 return HAL_OK;
}

/**
  * @brief  DeInitializes the PCD peripheral 
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DeInit(PCD_HandleTypeDef *hpcd)
{
  /* Check the PCD handle allocation */
  if(hpcd == NULL)
  {
    return HAL_ERROR;
  }

  hpcd->State = HAL_PCD_STATE_BUSY;
  
  /* Stop Device */
  HAL_PCD_Stop(hpcd);
    
  /* DeInit the low level hardware */
  HAL_PCD_MspDeInit(hpcd);
  
  hpcd->State = HAL_PCD_STATE_RESET;
  
  return HAL_OK;
}

/**
  * @brief  Initializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
__weak void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
__weak void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_MspDeInit could be implemented in the user file
   */
}

/**
  * @}
  */

/** @addtogroup PCD_Exported_Functions_Group2
 *  @brief   Data transfers functions 
 *
@verbatim   
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to manage the PCD data 
    transfers.

@endverbatim
  * @{
  */
  
/**
  * @brief  Start The USB OTG Device.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd)
{ 
  /* Enabling DP Pull-Down bit to Connect internal pull-up on USB DP line */
  hpcd->Instance->BCDR |= USB_BCDR_DPPU;
  
  return HAL_OK;
}

/**
  * @brief  Stop The USB OTG Device.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *hpcd)
{ 
  __HAL_LOCK(hpcd); 
  
    /* disable all interrupts and force USB reset */
  hpcd->Instance->CNTR = USB_CNTR_FRES;
  
  /* clear interrupt status register */
  hpcd->Instance->ISTR = 0U;
  
  /* switch-off device */
  hpcd->Instance->CNTR = (USB_CNTR_FRES | USB_CNTR_PDWN);
  
  __HAL_UNLOCK(hpcd); 
  return HAL_OK;
}

/**
  * @brief  This function handles PCD interrupt request.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd)
{
  uint32_t wInterrupt_Mask = 0U;
  
  if (__HAL_PCD_GET_FLAG (hpcd, USB_ISTR_CTR))
  {
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the sub */
    PCD_EP_ISR_Handler(hpcd);
  }

  if (__HAL_PCD_GET_FLAG (hpcd, USB_ISTR_RESET))
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_RESET);
    HAL_PCD_ResetCallback(hpcd);
    HAL_PCD_SetAddress(hpcd, 0U);
  }

  if (__HAL_PCD_GET_FLAG (hpcd, USB_ISTR_PMAOVR))
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_PMAOVR);    
  }

  if (__HAL_PCD_GET_FLAG (hpcd, USB_ISTR_ERR))
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_ERR);
    hpcd->Instance->CNTR &= ~USB_ISTR_ERR;
  }

  if (__HAL_PCD_GET_FLAG (hpcd, USB_ISTR_WKUP))
  {
    
    hpcd->Instance->CNTR &= (uint16_t) ~(USB_CNTR_LPMODE);
    
    /*set wInterrupt_Mask global variable*/
    wInterrupt_Mask = USB_CNTR_CTRM  | USB_CNTR_WKUPM | USB_CNTR_SUSPM /* | USB_CNTR_ERRM */ \
      | USB_CNTR_SOFM | USB_CNTR_ESOFM | USB_CNTR_RESETM;

    /*Set interrupt mask*/
    hpcd->Instance->CNTR = wInterrupt_Mask;

    /* enable L1REQ interrupt */ 
    if (hpcd->Init.lpm_enable ==1)
    {
      wInterrupt_Mask |= USB_CNTR_L1REQM;
      
      /* Enable LPM support and enable ACK answer to LPM request*/
      USB_TypeDef *USBx = hpcd->Instance;
      hpcd->lpm_active = ENABLE;
      hpcd->LPM_State = LPM_L0;
      
      USBx->LPMCSR |= (USB_LPMCSR_LMPEN);
      USBx->LPMCSR |= (USB_LPMCSR_LPMACK);
    } 
    
    HAL_PCD_ResumeCallback(hpcd);
    
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_WKUP);     
  }
  
  if (__HAL_PCD_GET_FLAG (hpcd, USB_ISTR_SUSP))
  {    
    /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_SUSP);  
    
    /* Force low-power mode in the macrocell */
    hpcd->Instance->CNTR |= USB_CNTR_FSUSP;
    hpcd->Instance->CNTR |= USB_CNTR_LPMODE;
    
    if (__HAL_PCD_GET_FLAG (hpcd, USB_ISTR_WKUP) == 0U)
    {
      HAL_PCD_SuspendCallback(hpcd);
    }
  }
  
    /* Handle LPM Interrupt */ 
  if(__HAL_PCD_GET_FLAG(hpcd, USB_ISTR_L1REQ))
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_L1REQ);      
    if( hpcd->LPM_State == LPM_L0)
    {   
      /* Force suspend and low-power mode before going to L1 state*/
      hpcd->Instance->CNTR |= USB_CNTR_LPMODE;
      hpcd->Instance->CNTR |= USB_CNTR_FSUSP;
      
      hpcd->LPM_State = LPM_L1;
      hpcd->BESL = (hpcd->Instance->LPMCSR & USB_LPMCSR_BESL) >>2 ;  
      HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L1_ACTIVE);
    }
    else
    {
      HAL_PCD_SuspendCallback(hpcd);
    }
  }

  if (__HAL_PCD_GET_FLAG (hpcd, USB_ISTR_SOF))
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_SOF); 
    HAL_PCD_SOFCallback(hpcd);
  }

  if (__HAL_PCD_GET_FLAG (hpcd, USB_ISTR_ESOF))
  {
    /* clear ESOF flag in ISTR */
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_ESOF); 
  }
}

/**
  * @brief  Data out stage callbacks
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
 __weak void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */ 
}

/**
  * @brief  Data IN stage callbacks
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
 __weak void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */ 
}
/**
  * @brief  Setup stage callback
  * @param  hpcd: PCD handle
  * @retval None
  */
 __weak void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */ 
}

/**
  * @brief  USB Start Of Frame callbacks
  * @param  hpcd: PCD handle
  * @retval None
  */
 __weak void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */ 
}

/**
  * @brief  USB Reset callbacks
  * @param  hpcd: PCD handle
  * @retval None
  */
 __weak void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */ 
}


/**
  * @brief  Suspend event callbacks
  * @param  hpcd: PCD handle
  * @retval None
  */
 __weak void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */ 
}

/**
  * @brief  Resume event callbacks
  * @param  hpcd: PCD handle
  * @retval None
  */
 __weak void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */ 
}

/**
  * @brief  Incomplete ISO OUT callbacks
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
 __weak void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */ 
}

/**
  * @brief  Incomplete ISO IN  callbacks
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
 __weak void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */ 
}

/**
  * @brief  Connection event callbacks
  * @param  hpcd: PCD handle
  * @retval None
  */
 __weak void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */ 
}

/**
  * @brief  Disconnection event callbacks
  * @param  hpcd: PCD handle
  * @retval None
  */
 __weak void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */ 
}

/**
  * @}
  */
  

/** @addtogroup PCD_Exported_Functions_Group3
 *  @brief   management functions 
 *
@verbatim   
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to control the PCD data 
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Connect the USB device
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd); 
  
  /* Enabling DP Pull-Down bit to Connect internal pull-up on USB DP line */
  hpcd->Instance->BCDR |= USB_BCDR_DPPU;
  
  __HAL_UNLOCK(hpcd); 
  return HAL_OK;
}

/**
  * @brief  Disconnect the USB device
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd); 
  
  /* Disable DP Pull-Down bit*/
   hpcd->Instance->BCDR &= ((uint16_t) ~(USB_BCDR_DPPU));
  
  __HAL_UNLOCK(hpcd); 
  return HAL_OK;
}

/**
  * @brief  Set the USB Device address 
  * @param  hpcd: PCD handle
  * @param  address: new device address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address)
{
   __HAL_LOCK(hpcd); 

   if(address == 0U) 
   {
     /* set device address and enable function */
     hpcd->Instance->DADDR = USB_DADDR_EF;
   }
   else /* USB Address will be applied later */
   {
     hpcd->USB_Address = address;
   }

  __HAL_UNLOCK(hpcd);   
  return HAL_OK;
}
/**
  * @brief  Open and configure an endpoint
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @param  ep_mps: endpoint max packert size
  * @param  ep_type: endpoint type   
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type)
{
  HAL_StatusTypeDef  ret = HAL_OK;
  PCD_EPTypeDef *ep;
  
  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & 0x7FU];
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & 0x7FU];
  }
  ep->num   = ep_addr & 0x7FU;
  
  ep->is_in = (0x80U & ep_addr) != 0U;
  ep->maxpacket = ep_mps;
  ep->type = ep_type;
  
  __HAL_LOCK(hpcd); 

/* initialize Endpoint */
  switch (ep->type)
  {
  case PCD_EP_TYPE_CTRL:
    PCD_SET_EPTYPE(hpcd->Instance, ep->num, USB_EP_CONTROL);
    break;
  case PCD_EP_TYPE_BULK:
    PCD_SET_EPTYPE(hpcd->Instance, ep->num, USB_EP_BULK);
    break;
  case PCD_EP_TYPE_INTR:
    PCD_SET_EPTYPE(hpcd->Instance, ep->num, USB_EP_INTERRUPT);
    break;
  case PCD_EP_TYPE_ISOC:
    PCD_SET_EPTYPE(hpcd->Instance, ep->num, USB_EP_ISOCHRONOUS);
    break;
  } 
  
  PCD_SET_EP_ADDRESS(hpcd->Instance, ep->num, ep->num);
  
  if (ep->doublebuffer == 0U) 
  {
    if (ep->is_in)
    {
      /*Set the endpoint Transmit buffer address */
      PCD_SET_EP_TX_ADDRESS(hpcd->Instance, ep->num, ep->pmaadress);
      PCD_CLEAR_TX_DTOG(hpcd->Instance, ep->num);
      /* Configure NAK status for the Endpoint*/
      PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num, USB_EP_TX_NAK); 
    }
    else
    {
      /*Set the endpoint Receive buffer address */
      PCD_SET_EP_RX_ADDRESS(hpcd->Instance, ep->num, ep->pmaadress);
      /*Set the endpoint Receive buffer counter*/
      PCD_SET_EP_RX_CNT(hpcd->Instance, ep->num, ep->maxpacket);
      PCD_CLEAR_RX_DTOG(hpcd->Instance, ep->num);
      /* Configure VALID status for the Endpoint*/
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_VALID);
    }
  }
  /*Double Buffer*/
  else
  {
    /*Set the endpoint as double buffered*/
    PCD_SET_EP_DBUF(hpcd->Instance, ep->num);
    /*Set buffer address for double buffered mode*/
    PCD_SET_EP_DBUF_ADDR(hpcd->Instance, ep->num,ep->pmaaddr0, ep->pmaaddr1);
    
    if (ep->is_in==0U)
    {
      /* Clear the data toggle bits for the endpoint IN/OUT*/
      PCD_CLEAR_RX_DTOG(hpcd->Instance, ep->num);
      PCD_CLEAR_TX_DTOG(hpcd->Instance, ep->num);
      
      /* Reset value of the data toggle bits for the endpoint out*/
      PCD_TX_DTOG(hpcd->Instance, ep->num);
      
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_VALID);
      PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num, USB_EP_TX_DIS);
    }
    else
    {
      /* Clear the data toggle bits for the endpoint IN/OUT*/
      PCD_CLEAR_RX_DTOG(hpcd->Instance, ep->num);
      PCD_CLEAR_TX_DTOG(hpcd->Instance, ep->num);
      PCD_RX_DTOG(hpcd->Instance, ep->num);
      /* Configure DISABLE status for the Endpoint*/
      PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num, USB_EP_TX_DIS);
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_DIS);
    }
  } 
  
  __HAL_UNLOCK(hpcd);   
  return ret;
}


/**
  * @brief  Deactivate an endpoint
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{  
  PCD_EPTypeDef *ep;
  
  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & 0x7FU];
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & 0x7FU];
  }
  ep->num   = ep_addr & 0x7FU;
  
  ep->is_in = (0x80U & ep_addr) != 0U;
  
  __HAL_LOCK(hpcd); 

  if (ep->doublebuffer == 0U) 
  {
    if (ep->is_in)
    {
      PCD_CLEAR_TX_DTOG(hpcd->Instance, ep->num);
      /* Configure DISABLE status for the Endpoint*/
      PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num, USB_EP_TX_DIS); 
    }
    else
    {
      PCD_CLEAR_RX_DTOG(hpcd->Instance, ep->num);
      /* Configure DISABLE status for the Endpoint*/
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_DIS);
    }
  }
  /*Double Buffer*/
  else
  { 
    if (ep->is_in==0U)
    {
      /* Clear the data toggle bits for the endpoint IN/OUT*/
      PCD_CLEAR_RX_DTOG(hpcd->Instance, ep->num);
      PCD_CLEAR_TX_DTOG(hpcd->Instance, ep->num);
      
      /* Reset value of the data toggle bits for the endpoint out*/
      PCD_TX_DTOG(hpcd->Instance, ep->num);
      
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_DIS);
      PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num, USB_EP_TX_DIS);
    }
    else
    {
      /* Clear the data toggle bits for the endpoint IN/OUT*/
      PCD_CLEAR_RX_DTOG(hpcd->Instance, ep->num);
      PCD_CLEAR_TX_DTOG(hpcd->Instance, ep->num);
      PCD_RX_DTOG(hpcd->Instance, ep->num);
      /* Configure DISABLE status for the Endpoint*/
      PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num, USB_EP_TX_DIS);
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_DIS);
    }
  } 
  
  __HAL_UNLOCK(hpcd);   
  return HAL_OK;
}


/**
  * @brief  Receive an amount of data  
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @param  pBuf: pointer to the reception buffer   
  * @param  len: amount of data to be received
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  
 PCD_EPTypeDef *ep;
  
  ep = &hpcd->OUT_ep[ep_addr & 0x7FU];
  
  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;  
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 0U;
  ep->num = ep_addr & 0x7FU;

  /* Multi packet transfer*/
  if (ep->xfer_len > ep->maxpacket)
  {
    len=ep->maxpacket;
    ep->xfer_len-=len; 
  }
  else
  {
    len=ep->xfer_len;
    ep->xfer_len =0U;
  }
  
  /* configure and validate Rx endpoint */
  if (ep->doublebuffer == 0U) 
  {
    /*Set RX buffer count*/
    PCD_SET_EP_RX_CNT(hpcd->Instance, ep->num, len);
  }
  else
  {
    /*Set the Double buffer counter*/
    PCD_SET_EP_DBUF_CNT(hpcd->Instance, ep->num, ep->is_in, len);
  } 
  
  PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_VALID);

  return HAL_OK;
}

/**
  * @brief  Get Received Data Size
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @retval Data Size
  */
uint16_t HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  return hpcd->OUT_ep[ep_addr & 0x7FU].xfer_count;
}
/**
  * @brief  Send an amount of data  
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @param  pBuf: pointer to the transmission buffer   
  * @param  len: amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;
  uint16_t pmabuffer = 0U;
    
  ep = &hpcd->IN_ep[ep_addr & 0x7FU];
  
  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;  
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 1U;
  ep->num = ep_addr & 0x7FU;

  /*Multi packet transfer*/
  if (ep->xfer_len > ep->maxpacket)
  {
    len=ep->maxpacket;
    ep->xfer_len-=len; 
  }
  else
  {  
    len=ep->xfer_len;
    ep->xfer_len =0U;
  }
  
  /* configure and validate Tx endpoint */
  if (ep->doublebuffer == 0U) 
  {
    PCD_WritePMA(hpcd->Instance, ep->xfer_buff, ep->pmaadress, len);
    PCD_SET_EP_TX_CNT(hpcd->Instance, ep->num, len);
  }
  else
  {
    /*Set the Double buffer counter */
    PCD_SET_EP_DBUF_CNT(hpcd->Instance, ep->num, ep->is_in, len);
    
    /*Write the data to the USB endpoint*/
    if (PCD_GET_ENDPOINT(hpcd->Instance, ep->num)& USB_EP_DTOG_TX)
    {
      pmabuffer = ep->pmaaddr1;
    }
    else
    {
      pmabuffer = ep->pmaaddr0;
    }
    
    PCD_WritePMA(hpcd->Instance, ep->xfer_buff, pmabuffer, len);
    PCD_FreeUserBuffer(hpcd->Instance, ep->num, ep->is_in);
  }

  PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num, USB_EP_TX_VALID);
  
  return HAL_OK;
}

/**
  * @brief  Set a STALL condition over an endpoint
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;
   
  __HAL_LOCK(hpcd); 
   
  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & 0x7FU];
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr];
  }
  
  ep->is_stall = 1U;
  ep->num   = ep_addr & 0x7FU;
  ep->is_in = ((ep_addr & 0x80U) == 0x80U);
  
  if (ep->num == 0U)
  {
    /* This macro sets STALL status for RX & TX*/ 
    PCD_SET_EP_TXRX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_STALL, USB_EP_TX_STALL); 
  }
  else
  {
    if (ep->is_in)
    {
      PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num , USB_EP_TX_STALL); 
    }
    else
    {
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num , USB_EP_RX_STALL);
    }
  }
  __HAL_UNLOCK(hpcd); 
  
  return HAL_OK;
}

/**
  * @brief  Clear a STALL condition over in an endpoint
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;
  
  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & 0x7FU];
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr];
  }
  
  ep->is_stall = 0U;
  ep->num   = ep_addr & 0x7FU;
  ep->is_in = ((ep_addr & 0x80U) == 0x80U);
  
  __HAL_LOCK(hpcd); 
  
  if (ep->is_in)
  {
    PCD_CLEAR_TX_DTOG(hpcd->Instance, ep->num);
    PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num, USB_EP_TX_VALID);
  }
  else
  {
    PCD_CLEAR_RX_DTOG(hpcd->Instance, ep->num);
    PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_VALID);
  }
  __HAL_UNLOCK(hpcd); 
    
  return HAL_OK;
}

/**
  * @brief  Flush an endpoint
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{ 
  return HAL_OK;
}

/**
  * @brief  HAL_PCD_ActivateRemoteWakeup : active remote wakeup signalling
  * @param  hpcd: PCD handle
  * @retval status
*/
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd)
{
  if (hpcd->Init.lpm_enable ==1)
  {
    /* Apply L1 Resume */
    hpcd->Instance->CNTR |= USB_CNTR_L1RESUME;
  }
  else
  {
    /* Apply L2 Resume */
    hpcd->Instance->CNTR |= USB_CNTR_RESUME;
  }
  return (HAL_OK);
}

/**
  * @brief  HAL_PCD_DeActivateRemoteWakeup : de-active remote wakeup signalling
  * @param  hpcd: PCD handle
  * @retval status
  */
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd)
{
  if (hpcd->Init.lpm_enable ==1)
  {
    /* Release L1 Resume */
    hpcd->Instance->CNTR &= ~ USB_CNTR_L1RESUME;
  }
  else
  {
    /* Release L2 Resume */
    hpcd->Instance->CNTR &= ~ USB_CNTR_RESUME;
  }
  return (HAL_OK);
}

/**
  * @}
  */

/** @addtogroup PCD_Exported_Functions_Group4
 *  @brief   Peripheral State functions
 *
@verbatim
 ===============================================================================
                      ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection permit to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the PCD state
  * @param  hpcd: PCD handle
  * @retval HAL state
  */
PCD_StateTypeDef HAL_PCD_GetState(PCD_HandleTypeDef *hpcd)
{
  return hpcd->State;
}
/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup PCD_Private
  * @{
  */


/**
  * @brief Copy a buffer from user memory area to packet memory area (PMA)
  * @param   USBx: USB device
  * @param   pbUsrBuf: pointer to user memory area.
  * @param   wPMABufAddr: address into PMA.
  * @param   wNBytes: no. of bytes to be copied.
  * @retval None
  */
void PCD_WritePMA(USB_TypeDef  *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes)
{
  uint16_t *pwPMABuf;
  uint16_t data;
  uint32_t i;
  uint32_t n;

  pwPMABuf = (uint16_t *)(wPMABufAddr + (uint32_t)USBx + 0x400U);
  n = (wNBytes + 1) >> 1U;

  if ((uint32_t)pbUsrBuf & 1)
  {
    for (i = 0; i < n; ++i)
    {
      data = pbUsrBuf[0] | (pbUsrBuf[1] << 8);
      pbUsrBuf += 2;
      pwPMABuf[i] = data;
    }
  }
  else
  {
    uint16_t *pwUsrBuf = (uint16_t *) pbUsrBuf;

    for (i = 0; i < n; ++i)
    {
      pwPMABuf[i] = pwUsrBuf[i];
    }
  }
}

/**
  * @brief Copy a buffer from user memory area to packet memory area (PMA)
  * @param   USBx: USB device
  * @param   pbUsrBuf: pointer to user memory area.
  * @param   wPMABufAddr: address into PMA.
  * @param   wNBytes: no. of bytes to be copied.
  * @retval None
  */
void PCD_ReadPMA(USB_TypeDef  *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes)
{
  uint16_t *pwPMABuf;
  uint16_t data;
  uint32_t i;
  uint32_t n;

  pwPMABuf = (uint16_t *)(wPMABufAddr + (uint32_t)USBx + 0x400U);
  n = wNBytes >> 1U;

  if ((uint32_t)pbUsrBuf & 1)
  {
    for (i = 0; i < n; ++i)
    {
      data = pwPMABuf[i];
      *pbUsrBuf++ = data & 0xFFu;
      *pbUsrBuf++ = data >> 8;
    }
    if (wNBytes & 1)
    {
      data = pwPMABuf[i];
      *pbUsrBuf = data & 0xFFu;
    }
  }
  else
  {
    uint16_t *pwUsrBuf = (uint16_t *) pbUsrBuf;

    for (i = 0; i < n; ++i)
    {
      pwUsrBuf[i] = pwPMABuf[i];
    }
    if (wNBytes & 1)
    {
      pbUsrBuf = (uint8_t *) &pwUsrBuf[i];
      data = pwPMABuf[i];
      *pbUsrBuf = data & 0xFFu;
    }
  }
}
/**
  * @brief  This function handles PCD Endpoint interrupt request.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd)
{
  PCD_EPTypeDef *ep;
  uint16_t count=0U;
  uint8_t EPindex;
  __IO uint16_t wIstr;  
  __IO uint16_t wEPVal = 0U;
  
  /* stay in loop while pending interrupts */
  while (((wIstr = hpcd->Instance->ISTR) & USB_ISTR_CTR) != 0U)
  {
    /* extract highest priority endpoint number */
    EPindex = (uint8_t)(wIstr & USB_ISTR_EP_ID);
    
    if (EPindex == 0U)
    {
      /* Decode and service control endpoint interrupt */
      
      /* DIR bit = origin of the interrupt */   
      if ((wIstr & USB_ISTR_DIR) == 0U)
      {
        /* DIR = 0 */
        
        /* DIR = 0      => IN  int */
        /* DIR = 0 implies that (EP_CTR_TX = 1) always  */
        PCD_CLEAR_TX_EP_CTR(hpcd->Instance, PCD_ENDP0);
        ep = &hpcd->IN_ep[0];
        
        ep->xfer_count = PCD_GET_EP_TX_CNT(hpcd->Instance, ep->num);
        ep->xfer_buff += ep->xfer_count;
 
        /* TX COMPLETE */
        HAL_PCD_DataInStageCallback(hpcd, 0U);
        
        
        if((hpcd->USB_Address > 0U)&& ( ep->xfer_len == 0U))
        {
          hpcd->Instance->DADDR = (hpcd->USB_Address | USB_DADDR_EF);
          hpcd->USB_Address = 0U;
        }
        
      }
      else
      {
        /* DIR = 1 */
        
        /* DIR = 1 & CTR_RX       => SETUP or OUT int */
        /* DIR = 1 & (CTR_TX | CTR_RX) => 2 int pending */
        ep = &hpcd->OUT_ep[0];
        wEPVal = PCD_GET_ENDPOINT(hpcd->Instance, PCD_ENDP0);
        
        if ((wEPVal & USB_EP_SETUP) != 0U)
        {
          /* Get SETUP Packet*/
          ep->xfer_count = PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);
          PCD_ReadPMA(hpcd->Instance, (uint8_t*)hpcd->Setup ,ep->pmaadress , ep->xfer_count);       
          /* SETUP bit kept frozen while CTR_RX = 1*/ 
          PCD_CLEAR_RX_EP_CTR(hpcd->Instance, PCD_ENDP0); 
          
          /* Process SETUP Packet*/
          HAL_PCD_SetupStageCallback(hpcd);
        }
        
        else if ((wEPVal & USB_EP_CTR_RX) != 0U)
        {
          PCD_CLEAR_RX_EP_CTR(hpcd->Instance, PCD_ENDP0);
          /* Get Control Data OUT Packet*/
          ep->xfer_count = PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);
          
          if (ep->xfer_count != 0U)
          {
            PCD_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaadress, ep->xfer_count);
            ep->xfer_buff+=ep->xfer_count;
          }
          
          /* Process Control Data OUT Packet*/
           HAL_PCD_DataOutStageCallback(hpcd, 0U);
          
          PCD_SET_EP_RX_CNT(hpcd->Instance, PCD_ENDP0, ep->maxpacket);
          PCD_SET_EP_RX_STATUS(hpcd->Instance, PCD_ENDP0, USB_EP_RX_VALID);
        }
      }
    }
    else
    {
      
      /* Decode and service non control endpoints interrupt  */
      
      /* process related endpoint register */
      wEPVal = PCD_GET_ENDPOINT(hpcd->Instance, EPindex);
      if ((wEPVal & USB_EP_CTR_RX) != 0U)
      {  
        /* clear int flag */
        PCD_CLEAR_RX_EP_CTR(hpcd->Instance, EPindex);
        ep = &hpcd->OUT_ep[EPindex];
        
        /* OUT double Buffering*/
        if (ep->doublebuffer == 0U)
        {
          count = PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);
          if (count != 0U)
          {
            PCD_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaadress, count);
          }
        }
        else
        {
          if (PCD_GET_ENDPOINT(hpcd->Instance, ep->num) & USB_EP_DTOG_RX)
          {
            /*read from endpoint BUF0Addr buffer*/
            count = PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);
            if (count != 0U)
            {
              PCD_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr0, count);
            }
          }
          else
          {
            /*read from endpoint BUF1Addr buffer*/
            count = PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);
            if (count != 0U)
            {
              PCD_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr1, count);
            }
          }
          PCD_FreeUserBuffer(hpcd->Instance, ep->num, PCD_EP_DBUF_OUT);  
        }
        /*multi-packet on the NON control OUT endpoint*/
        ep->xfer_count+=count;
        ep->xfer_buff+=count;
       
        if ((ep->xfer_len == 0U) || (count < ep->maxpacket))
        {
          /* RX COMPLETE */
          HAL_PCD_DataOutStageCallback(hpcd, ep->num);
        }
        else
        {
          HAL_PCD_EP_Receive(hpcd, ep->num, ep->xfer_buff, ep->xfer_len);
        }
        
      } /* if((wEPVal & EP_CTR_RX) */
      
      if ((wEPVal & USB_EP_CTR_TX) != 0U)
      {
        ep = &hpcd->IN_ep[EPindex];
        
        /* clear int flag */
        PCD_CLEAR_TX_EP_CTR(hpcd->Instance, EPindex);
        
        /* IN double Buffering*/
        if (ep->doublebuffer == 0U)
        {
          ep->xfer_count = PCD_GET_EP_TX_CNT(hpcd->Instance, ep->num);
          if (ep->xfer_count != 0U)
          {
            PCD_WritePMA(hpcd->Instance, ep->xfer_buff, ep->pmaadress, ep->xfer_count);
          }
        }
        else
        {
          if (PCD_GET_ENDPOINT(hpcd->Instance, ep->num) & USB_EP_DTOG_TX)
          {
            /*read from endpoint BUF0Addr buffer*/
            ep->xfer_count = PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);
            if (ep->xfer_count != 0U)
            {
              PCD_WritePMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr0, ep->xfer_count);
            }
          }
          else
          {
            /*read from endpoint BUF1Addr buffer*/
            ep->xfer_count = PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);
            if (ep->xfer_count != 0U)
            {
              PCD_WritePMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr1, ep->xfer_count);
            }
          }
          PCD_FreeUserBuffer(hpcd->Instance, ep->num, PCD_EP_DBUF_IN);  
        }
        /*multi-packet on the NON control IN endpoint*/
        ep->xfer_count = PCD_GET_EP_TX_CNT(hpcd->Instance, ep->num);
        ep->xfer_buff+=ep->xfer_count;
       
        /* Zero Length Packet? */
        if (ep->xfer_len == 0U)
        {
          /* TX COMPLETE */
          HAL_PCD_DataInStageCallback(hpcd, ep->num);
        }
        else
        {
          HAL_PCD_EP_Transmit(hpcd, ep->num, ep->xfer_buff, ep->xfer_len);
        }
      } 
    }
  }
  return HAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* HAL_PCD_MODULE_ENABLED */
#endif /* #if !defined(STM32L011xx) && !defined(STM32L021xx) && !defined (STM32L031xx) && !defined (STM32L041xx) && !defined (STM32L051xx) && !defined (STM32L061xx) && !defined (STM32L071xx) && !defined (STM32L081xx) */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

