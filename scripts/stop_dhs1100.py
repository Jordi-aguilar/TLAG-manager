import sys
import time
sys.path.insert(1, "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA")

from instruments import DHS1100_rtk


def main():
    while 1:
        try:
            dhs1100 = DHS1100_rtk()

            while 1:
                try:
                    dhs1100.write_setpoint(25)
                    dhs1100.write_temp_control(0)
                    dhs1100.write_target_power(0)
                    time.sleep(0.2)
                except Exception as e:
                    print(e)


        except Exception as e:
            print(e)
            time.sleep(0.2)


if __name__ == "__main__":
    main()