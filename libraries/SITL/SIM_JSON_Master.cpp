/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Send and receve JSON backend data to alow a second AP instance to ride along
*/

#include "SIM_JSON_Master.h"
#include <AP_Logger/AP_Logger.h>

using namespace SITL;

JSON_Master::JSON_Master(const int32_t &num_slaves)
{
    socket_list *list = &_list;
    for (uint8_t i = 1; i <= num_slaves; i++) {
        // init each socket and instance
        if (list == nullptr) {
            list = new socket_list;
        }
        list->instance = i;

        list->sock_in.set_blocking(false);
        list->sock_in.reuseaddress();

        list->sock_out.set_blocking(false);
        list->sock_out.reuseaddress();

        uint16_t port = 9002 + 10 * i;
        list->sock_in.bind("127.0.0.1", port);
        printf("Slave %u: listening on %u\n", i, port);
    }
}

// Receve PWM outs from ride along controlers
void JSON_Master::receve(struct sitl_input &input)
{
    uint8_t master_instance = AP::sitl()->ride_along_master.get();

    for (socket_list *list = &_list; list; list=list->next) {
        // cycle through all ride along instances
        struct servo_packet {
            uint16_t magic;
            uint16_t frame_rate;
            uint32_t frame_count;
            uint16_t pwm[16];
        } buffer;

        while (true) {
            ssize_t ret = list->sock_in.recv(&buffer, sizeof(buffer), 100);
            if (ret == 0) {
                // wait some more
                continue;
            }
            if (buffer.magic != 18458) {
                // magic value does not match
                continue;
            }
            break;
        }
        if (!list->connected) {
            // connect back to the last addres for send
            uint16_t port;
            list->sock_in.last_recv_address(_ip, port);
            list->sock_out.connect(_ip, port);
            list->connected = true;
            printf("Slave %u connected to %s:%u\n", list->instance, _ip, port);
        }

// @LoggerMessage: SLV1
// @Description: Log data received from JSON simulator 1
// @Field: TimeUS: Time since system startup (us)
// @Field: Instance: Slave instance
// @Field: frame_rate: Slave instance's desired frame rate
// @Field: frame_count: Slave instance's current frame count
        AP::logger().Write("SLV1", "TimeUS,Instance,magic,frame_rate,frame_count",
                       "s#---",
                       "F????",
                       "QBHHI",
                       AP_HAL::micros64(),
                       list->instance,
                       buffer.magic,
                       buffer.frame_rate,
                       buffer.frame_count);

// @LoggerMessage: SLV2
// @Description: Log data received from JSON simulator 2
// @Field: TimeUS: Time since system startup
// @Field: Instance: Slave instance
// @Field: C1: channel 1 output
// @Field: C2: channel 2 output
// @Field: C3: channel 3 output
// @Field: C4: channel 4 output
// @Field: C5: channel 5 output
// @Field: C6: channel 6 output
// @Field: C7: channel 7 output
// @Field: C8: channel 8 output
// @Field: C9: channel 9 output
// @Field: C10: channel 10 output
// @Field: C11: channel 11 output
// @Field: C12: channel 12 output
// @Field: C13: channel 13 output
// @Field: C14: channel 14 output
        AP::logger().Write("SLV2", "TimeUS,Instance,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14",
                       "s#YYYYYYYYYYYYYY",
                       "F?--------------",
                       "QBHHHHHHHHHHHHHH",
                       AP_HAL::micros64(),
                       list->instance,
                       buffer.pwm[0],
                       buffer.pwm[1],
                       buffer.pwm[2],
                       buffer.pwm[3],
                       buffer.pwm[4],
                       buffer.pwm[5],
                       buffer.pwm[6],
                       buffer.pwm[7],
                       buffer.pwm[8],
                       buffer.pwm[9],
                       buffer.pwm[10],
                       buffer.pwm[11],
                       buffer.pwm[12],
                       buffer.pwm[13],
                       buffer.pwm[14]);

        if (list->instance == master_instance) {
            // Use the servo outs from this instance
            memcpy(input.servos,buffer.pwm,sizeof(input.servos));
        }
    }
}

// send vehicle state to ride along controlers
void JSON_Master::send(const struct sitl_fdm &output, const Vector3f &position)
{
    // message is the same to all slaves
    char *send_buffer = nullptr;
    int length = asprintf(&send_buffer,"\n{\"timestamp\":%f,\"imu\":{\"gyro\":[%f,%f,%f],\"accel_body\":[%f,%f,%f]},\"position\":[%f,%f,%f],\"quaternion\":[%f,%f,%f,%f],\"velocity\":[%f,%f,%f],\"no_time_sync\":1}\n",
                            output.timestamp_us * 10e-6,
                            radians(output.rollRate), radians(output.pitchRate), radians(output.yawRate),
                            output.xAccel, output.yAccel, output.zAccel,
                            position.x, position.y, position.z,
                            output.quaternion.q1, output.quaternion.q2, output.quaternion.q3, output.quaternion.q4,
                            output.speedN, output.speedE, output.speedD);

    for (socket_list *list = &_list; list; list=list->next) {
        list->sock_out.send(send_buffer,length);
    }
}
