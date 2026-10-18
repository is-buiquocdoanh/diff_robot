# Quy trình tune PID

Tài liệu đầy đủ (firmware tune riêng + web tool, cách đọc đồ thị, chỉ số
overshoot/rise time/settle time, mang bộ số về sản xuất) nằm ở project
[`src/diff_tune_PID`](../diff_tune_PID/):
**[src/diff_tune_PID/Quy_trinh_tune_PID.md](../diff_tune_PID/Quy_trinh_tune_PID.md)**.

Firmware sản xuất này (`diff_drive_ros`) đã gộp sẵn giao thức tune (`T,1`/`T,0`,
`V,...`, `K,...`) nên có thể tune trực tiếp không cần nạp firmware riêng — xem
[README.md mục 13](README.md#13-tune-pid-bằng-web-tool-không-cần-firmware-riêng).
