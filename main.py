from pymavlink.mavextra import euler_to_quat

from control import Vehicle

if __name__ == "__main__":
    print(euler_to_quat([0, 0, 1]))
    exit()
    v = Vehicle(speedup=5)
    v.connect("tcp:localhost:14540")
    # v.arm()
    # v.takeoff()
    # v.climb(500)
    v.set_mode("GUIDED")
    for _ in range(30):
        v._do_fom_edge_att_rate()
    # v.disarm()
