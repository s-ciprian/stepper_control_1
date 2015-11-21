# stepper_control_1
Evaluation ST Microelectronics motor control solution - hardware and software library.


Project based on ST hardware kit and software library for stepper motor control.
Controls one motor.

Hardware:
 - microcontroller board: Nucleo F401RE (microcontroller STM32F401RET6)
   http://www.st.com/web/catalog/tools/FM116/SC959/SS1532/LN1847/PF260000?icmp=nucleo-ipf_pron_pr-nucleo_feb2014&sc=nucleoF401RE-pr
 - motor driver board: X-Nucleo-IHM01A1 (stepper motor driver IC L6474PD)
   http://www.st.com/web/en/catalog/tools/PF260715
 - Wantai stepper motor: model 42BYGHW609
   http://www.wantmotor.com/ProductsView.asp?id=155&pid=80
 
Software:
 - ST library for motor control: X-CUBE-SPN1 (Stepper motor driver software expansion for STM32Cube)
   http://www.st.com/web/en/catalog/tools/PF261441
   Project is based on example: X-CUBE-SPN1-20150128\stm32_cube\Projects\Multi\Examples\MotionControl\IHM01A1_ExampleFor1Motor
 - IDE: Atollic TrueStudio 5.4.0 Free Edition (no size code limitation and no registration is required for download)
   http://timor.atollic.com/truestudio/
