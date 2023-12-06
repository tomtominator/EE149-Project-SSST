# EE149-Project-SSST

UART: 
Tested using ESP32 CAM(slave) and NodeMCU ESP32(master).
NodeMCU used UART2 (pins 16,17). ESP32 CAM used its primary (and only UART, which is also used for flashing)(pins 1,3).

To upload when using primary uart pin GPIO0 must be brought low(done automatically by the micro-usb extension board).

Next test is to have ESP32 CAM as master and ardunio Uno as slave.

-----

Links: 
Drone construction guide: https://www.brokking.net/ymfc-al_main.html  

149 Part Checkout: https://docs.google.com/spreadsheets/d/1Bz1YIEwWmc2XcUH2Z1pXMXEvFyvBWZiJ78gy9sDLGXI/edit#
Purchase Request: https://forms.gle/wH2baykkZNVtYD1u6  


Proposal Doc: https://docs.google.com/document/d/1jsfoBy7dJqcI1VUTb40JnPD3tMkyC29lHxZ-uyRWZ1Q/edit  

Ideas:  
- Drone Based:
  - 3rd person drone
  - flight controller in drone
- Car based:
  - line following robot
  - drawing robot
  - ADAS bot
- Non Vehicle:
  - guitar playing robot
  - voice transcriber
