/**
  @page Template Description of the Templates project
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    Project/STM32F4xx_StdPeriph_Templates/readme.txt 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Description of the TEMPLATE example
  ******************************************************************************
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the Licens
  *   
  ******************************************************************************
  @endverbatim

@par Example Description

This example is used as a template project that can be used as reference to build
any new firmware application for STM32F405xx/407xx, STM32F415xx/417xx, STM32F427xx/437xx, 
STM32F429xx/439xx, STM32F401xx/410xx/411xE, STM32F412xx, STM32F446xx, STM32F469xx/479xx 
or STM32F413_423xx devices 
using the STM32F4xx Standard Peripherals Library.

@par Directory contents

  - Template/system_stm32f4xx.c   STM32F4xx system clock configuration file
  - Template/stm32f4xx_conf.h     Library Configuration file
  - Template/stm32f4xx_it.c       Interrupt handlers
  - Template/stm32f4xx_it.h       Interrupt handlers header file
  - Template/main.c               Main program
  - Template/main.h               Main program header file

@par Hardware and Software environment

  - This example runs on STM32F405xx/407xx, STM32F415xx/417xx, STM32F427xx/437xx, 
    STM32F429xx/439xx, STM32F401xx/410xx/411xE,STM32F412xx, STM32F446xx 
    and STM32F469xx/479xx and STM32F413_423xx devices.

  - This example has been tested with STMicroelectronics STM324xG-EVAL (STM32F40xx/
    STM32F41xx Devices),STM32F410xx-Nucleo RevC (STM32F410xx), STM32F412G-Discovery RevC
    (STM32F412xx), STM32F413H-DISCO revA (STM32F413_423xx), STM32437I-EVAL (STM32F427xx/STM32F437xx Devices), 
    STM324x9I-EVAL RevB (STM32F429xx/STM32F439xx Devices), STM32446E-EVAL RevB (STM32F446xx Devices) 
    and STM32F469I-EVAL RevB (STM32F469xx Devices) evaluation boards.
    This example can be easily tailored to any other supported device and development board.

@par How to use it ? 

In order to make the program work, you must do the following:
 + EWARM
    - Open the Template.eww workspace 
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)
 
 + MDK-ARM
    - Open the Template.uvprojx project
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5) 
    
 + TrueSTUDIO
    - Open the TrueSTUDIO toolchain.
    - Click on File->Switch Workspace->Other and browse to TrueSTUDIO workspace directory.
    - Click on File->Import, select General->'Existing Projects into Workspace' and then click "Next". 
    - Browse to the TrueSTUDIO workspace directory, select the project.
    - Rebuild all project files: Select the project in the "Project explorer" 
      window then click on Project->build project menu.
    - Run program: Run->Debug (F11)
 + SW4STM32
    - Open the SW4STM32 toolchain.
    - Click on File->Switch Workspace->Other and browse to SW4STM32 workspace directory.
    - Click on File->Import, select General->'Existing Projects into Workspace' and then click "Next". 
    - Browse to the SW4STM32 workspace directory, select the project.
    - Rebuild all project files: Select the project in the "Project explorer" 
      window then click on Project->build project menu.
    - Run program: Run->Debug (F11)

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 