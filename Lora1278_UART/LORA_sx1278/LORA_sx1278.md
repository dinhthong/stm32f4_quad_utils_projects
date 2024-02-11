Config mode:

https://iotmaker.vn/module-lora-sx1278-433mhz.html

Hướng dẫn sử dụng module:

Để board hoạt động truyền nhận bình thường ta cần set 2 chân M0 và M1 về mức 0, ngoài ra ta có thể kết nối 2 chân này với 2 chân GPIO của vi điều khiển để cài đặt các chế độ hoạt động của module, bạn có thể tham khảo bảng sau:

Working mode	M1	M0	Introduction
Mode 0 normal	0	0	Serial open and wireless open, transparent transmission
Mode 1 Wake-up	0	1	Serial open and wireless open
Mode 2 Power-saving	1	0	Serial close and wireless wake-up mode
Mode 3 Sleep	1	1	Sleep, and can receive parameter setting command