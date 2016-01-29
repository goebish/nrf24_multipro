# nrf24-multipro
nRF24L01 multi-protocol RC transmitter library

![Screenshot](http://i.imgur.com/AeMJKzT.jpg)

Create a model in OpenTX using external module in PPM mode, 12 channels and TAER sequence order.

#####Protocol is selected with stick position at startup:

Elevator down + Aileron left = Syma X5C-1/X11/X11C/X12  
Elevator down + Aileron right = Attop YD-822/YD-829/YD-829C ...  
Elevator up + Aileron right = EAchine H8 mini, BayangToys X6/X7/X9, JJRC JJ850 ...  
Elevator up + Aileron left = EAchine H7  
Elevator up = WLToys V202/252/272, JXD 385/388, JJRC H6C, Yizhan Tarantula X6 ...  
Elevator down = EAchine CG023/CG031/3D X4  
Aileron left = Cheerson CX-10 green pcb  
Aileron right = Cheerson CX-10 blue pcb & newer red pcb, CX-10A, CX11, CX12  

Last used protocol is automatically selected if stick is in neutral position.

#####Extra features (if available on aircraft):

Channel 5: led light (or 3 pos. rate on CX-10, H7)  
Channel 6: flip control  
Channel 7: still camera  
Channel 8: video camera  
Channel 9: headless  
Channel 10: calibrate Y (V2x2), pitch trim (H7), RTH (H8 mini)  
Channel 11: calibrate X (V2x2), roll trim (H7)  
Channel 12: Reset / Rebind  
