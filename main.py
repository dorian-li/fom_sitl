from control import Vehicle

if __name__ == "__main__":
    v = Vehicle(speedup=5)
    v.connect("tcp:127.0.0.1:14540")
    # v.arm()
    # v.takeoff()
    v.set_mode("GUIDED")
    for _ in range(30):
        v._do_fom_edge_att_rate()
    # v.disarm()
