#include <Pid.h>
#include <mbed.h>

#include <algorithm>

// 速度コントローラID 1~8まで
constexpr unsigned motor_id = 5;
// 目標rpm -9000~9000rpmまで
constexpr int target_rpm = 8000;

BufferedSerial pc{USBTX, USBRX, 115200};
CAN can{PA_11, PA_12, (int)1e6};
// CAN can{PB_12, PB_13, (int)1e6};
Timer timer;

// c620から受け取るデータ
struct Receive {
  // ctor
  Receive() = default;
  Receive(uint8_t data[8])
      : angle{uint16_t(data[0] << 8 | data[1])}
      , rpm{int16_t(data[2] << 8 | data[3])}
      , ampere{int16_t(data[4] << 8 | data[5])}
      , temp{data[6]} {}
  Receive(const Receive&) = default;

  uint16_t angle;
  int16_t rpm;
  int16_t ampere;
  uint8_t temp;
  uint8_t padding;
};

uint8_t raw[8];
Receive rec;
rct::Pid<int> pid{{0.8f, 0.5f}};

int main() {
  // put your setup code here, to run once:
  printf("\nsetup\n");

  timer.start();
  auto pre = timer.elapsed_time();
  while(1) {
    // put your main code here, to run repeatedly:
    auto now = timer.elapsed_time();
    int out = pid.calc(8000, rec.rpm, now - pre);
    // out -16000~16000まで
    out = std::clamp(out, -16000, 16000);
    // out = 16000;
    pre = now;

    for(int i = 0; i < 4; ++i) {
      raw[i] = (out >> 8) & 0xff;
      raw[i + 1] = out & 0xff;
    }

    // int16_t data[4] = {5000, 5000, 5000, 5000};
    // for(int i = 0; i < 4; ++i) {
    //   raw[i] = (data[i] >> 8) & 0xff;
    //   raw[i + 1] = data[i] & 0xff;
    // }

    {
      CANMessage msg{0x200, raw, 8};
      if(!can.write(msg)) printf("failed");
    }
    CANMessage msg{0x1FF, raw, 8};
    if(!can.write(msg)) printf("failed");
    if(can.read(msg)) {
      if(msg.id == (0x200u | motor_id)) {
        printf("%x\t", msg.id);
        printf("%d\t", out);
        rec = Receive{msg.data};
        printf("%d\t", rec.angle);
        printf("%d\t", rec.rpm);
        printf("%d\t", rec.ampere);
        printf("%d\n", rec.temp);
      } else {
        printf("%d\n", msg.id);
      }
    } else {
      printf("no msg\n");
    }

    ThisThread::sleep_for(10ms);
  }
}
