## Connections:
PL2303: rx tx to LORA: 3tx 4(rx) // RX <-> TX

LORA and MCU: rx <-> tx

Pixhawk PL2303
chân gần dây Vcc ( đỏ ) nhất - RX
-> Conclusions:
Pixhawk LORA: rx <-> tx
LORA: tx:
## STM32 


#Config mode:

https://iotmaker.vn/module-lora-sx1278-433mhz.html

Hướng dẫn sử dụng module:

Để board hoạt động truyền nhận bình thường ta cần set 2 chân M0 và M1 về mức 0, ngoài ra ta có thể kết nối 2 chân này với 2 chân GPIO của vi điều khiển để cài đặt các chế độ hoạt động của module, bạn có thể tham khảo bảng sau:

Working mode	M1	M0	Introduction
Mode 0 normal	0	0	Serial open and wireless open, transparent transmission
Mode 1 Wake-up	0	1	Serial open and wireless open
Mode 2 Power-saving	1	0	Serial close and wireless wake-up mode
Mode 3 Sleep	1	1	Sleep, and can receive parameter setting command

## Links

- http://hshop.vn/products/mach-thu-phat-rf-lora-uart-sx1278-433mhz3000m
- https://vi.aliexpress.com/item/433-m-wireless-serial-interface-module-LORA-spread-spectrum-SX1278-1276-passthrough-5000-m-Arduino-recommended/32627590559.html?spm=a2g14.search0302.3.25.5846292dTVEpSr&ws_ab_test=searchweb0_0,searchweb201602_0_10152_10151_10618_10059_10696_10084_100031_10083_10547_10624_10623_10307_10548_10341_10065_10340_10068_10343_10342_308_10103_10620_10344_10622_10621,searchweb201603_0,ppcSwitch_0&algo_pvid=ffe293c9-a59b-49d8-a03d-d55f4f4d9b25&algo_expid=ffe293c9-a59b-49d8-a03d-d55f4f4d9b25-3
