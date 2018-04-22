
typedef struct __attribute__((__packed__)) radio_packet_t
{ //should be 15 bytes
    unsigned packet_num : 18; //data from skybass
    unsigned altitude : 15;
    unsigned sb_state : 4;
    unsigned battery : 8;
    unsigned lat : 18;
    unsigned lon : 18;
    unsigned gps_lock : 1;
    unsigned strato_alt : 15; //data from stratologger
    unsigned vsense1 : 8;   //voltage divider data
    unsigned vsense2 : 8;   
    unsigned skybass_alive : 1; //data from ESPs
    unsigned skybass_armed : 1;
} radio_packet_t;