# Quy ước vị trí kệ và luồng hàng

## Mã vị trí

Mã có dạng `<kệ><tầng><cột>`. Tầng và cột đánh số từ 0: tầng 0 là tầng thấp, tầng 1 là tầng cao; cột 0 đến 2 đi từ trái sang phải khi nhìn từ lối đi vào kệ. `A00` vì vậy là kệ A, tầng thấp, cột trái.

Mã ngắn chỉ dùng để hiển thị và trao đổi với người vận hành. ROS, ArUco và SQLite tiếp tục dùng ID kỹ thuật hiện tại (`A-0-1`, tag `0`, v.v.) để không làm thay đổi các nhiệm vụ hoặc dữ liệu đang có.

| Mã vị trí | Kệ | Tầng | Cột | Tag ô | Tâm khay hàng (x, y, z) m | Pose robot thao tác (x, y) m |
|---|---|---:|---|---:|---|---|
| A00 | A | Thấp | Trái | 0 | (0.75, 1.87, 0.171) | (0.75, 1.325) |
| A01 | A | Thấp | Giữa | 1 | (1.25, 1.87, 0.171) | (1.25, 1.325) |
| A02 | A | Thấp | Phải | 2 | (1.75, 1.87, 0.171) | (1.75, 1.325) |
| A10 | A | Cao | Trái | 3 | (0.75, 1.87, 0.421) | (0.75, 1.325) |
| A11 | A | Cao | Giữa | 4 | (1.25, 1.87, 0.421) | (1.25, 1.325) |
| A12 | A | Cao | Phải | 5 | (1.75, 1.87, 0.421) | (1.75, 1.325) |
| B00 | B | Thấp | Trái | 6 | (0.75, -1.87, 0.171) | (0.75, -1.325) |
| B01 | B | Thấp | Giữa | 7 | (1.25, -1.87, 0.171) | (1.25, -1.325) |
| B02 | B | Thấp | Phải | 8 | (1.75, -1.87, 0.171) | (1.75, -1.325) |
| B10 | B | Cao | Trái | 9 | (0.75, -1.87, 0.421) | (0.75, -1.325) |
| B11 | B | Cao | Giữa | 10 | (1.25, -1.87, 0.421) | (1.25, -1.325) |
| B12 | B | Cao | Phải | 11 | (1.75, -1.87, 0.421) | (1.75, -1.325) |

## Điểm nhận và điểm giao hàng

Hai trạm là điểm thao tác riêng, không phải ô kệ. Mọi ô Axx/Bxx dùng cùng luồng trạm:

| Mã điểm | Vai trò | Tag | Tâm khay (x, y, z) m | Pose robot thao tác (x, y) m |
|---|---|---:|---|---|
| IN | Nhận hàng đầu vào; robot gắp tại đây rồi đưa tới ô kệ Axx/Bxx | 30 | (-2.40, 1.150, 0.171) | (-2.40, 0.605) |
| OUT | Giao/trả hàng ra; robot lấy từ ô kệ Axx/Bxx rồi mang tới đây | 31 | (-2.40, -1.150, 0.171) | (-2.40, -0.605) |

`staging_pose` là điểm chờ tiếp cận; `work_pose` là pose robot dùng để gắp hoặc đặt tại khay. Vì vậy luồng nhập là `IN -> Axx/Bxx`, còn luồng xuất là `Axx/Bxx -> OUT`. Khi xuất, ô nguồn cụ thể được chọn theo FIFO; điểm giao cuối luôn là `OUT`.