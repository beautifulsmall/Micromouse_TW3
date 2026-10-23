// Radio console TX ring (lib/HAL/HAL.cpp), lifted verbatim and driven
// against a fake UART with a small, stalling TX buffer.
//
// This exists because the FIRST version had no ring: hal_radio_write()
// took only what Serial5.availableForWrite() reported and threw the rest
// away. The Teensy TX buffer is a few dozen bytes, so every console line
// longer than that lost its tail -- silently, with no error anywhere. It
// surfaced on the floor as a scale test printing
//     scale test done: believed L 906.7  R 90
// where the R value and the entire correction line that followed were
// simply gone. The measurement was fine; the transport ate it.
//
// A dropped telemetry line is a nuisance. A dropped MEASUREMENT is a
// wrong answer, so the transport has to be lossless up to a real overrun.
//   g++ -std=c++17 test_radio_ring.cpp -o t && ./t

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
// The ring logic lifted verbatim, with a fake UART whose room is small and
// varies -- which is exactly the condition that ate the scale-test output.
static uint8_t radio_tx[1024];
static uint16_t radio_head=0, radio_tail=0;
static uint32_t radio_dropped=0;
static std::string uart_out;
static int uart_room = 40;                       // Teensy-ish TX buffer
static void uart_write(uint8_t b){ uart_out.push_back((char)b); }
static void hal_radio_poll(){
    int room = uart_room;
    while (room>0 && radio_tail!=radio_head){
        uart_write(radio_tx[radio_tail]);
        radio_tail = uint16_t((radio_tail+1)&(sizeof(radio_tx)-1));
        room--;
    }
}
static size_t hal_radio_write(const uint8_t*buf,size_t len){
    size_t took=0;
    for(size_t i=0;i<len;i++){
        uint16_t nxt=uint16_t((radio_head+1)&(sizeof(radio_tx)-1));
        if(nxt==radio_tail){ radio_dropped++; break; }
        radio_tx[radio_head]=buf[i]; radio_head=nxt; took++;
    }
    hal_radio_poll();
    return took;
}
static int fails=0;
static void expect(const char*w,bool ok){ if(!ok){ printf("FAIL: %s\n",w); fails++; } }
int main(){
    // 1. The exact line that got truncated on the floor.
    const char *l1="scale test: driving 900 mm (5.0 cells) straight -- measure physical travel, then circ_new = circ * phys / 900\n";
    const char *l2="scale test done: believed L 906.7  R 906.1 mm (cmd 900)\n";
    const char *l3="  measure PHYSICAL travel, then: circl = 67.82 * phys / 906.7 ; circr = 67.94 * phys / 906.1\n";
    uart_out.clear();
    for(const char*l:{l1,l2,l3}) hal_radio_write((const uint8_t*)l,strlen(l));
    for(int i=0;i<200;i++) hal_radio_poll();     // main loop passes
    std::string want=std::string(l1)+l2+l3;
    printf("  queued %zu bytes, UART received %zu\n",want.size(),uart_out.size());
    expect("nothing truncated", uart_out==want);
    expect("the R value survived", uart_out.find("R 906.1")!=std::string::npos);
    expect("the correction line survived", uart_out.find("circr = 67.94")!=std::string::npos);
    expect("nothing dropped", radio_dropped==0);

    // 2. Wraparound: push far more than the ring, draining as we go.
    uart_out.clear(); radio_dropped=0;
    std::string big; for(int i=0;i<5000;i++) big.push_back(char('a'+(i%26)));
    size_t sent=0;
    while(sent<big.size()){
        sent+=hal_radio_write((const uint8_t*)big.data()+sent,big.size()-sent);
        for(int i=0;i<4;i++) hal_radio_poll();
    }
    for(int i=0;i<500;i++) hal_radio_poll();
    expect("wraps without corruption", uart_out==big);
    printf("  wraparound: %zu bytes through a %zu-byte ring, dropped %u\n",
           big.size(),sizeof(radio_tx),radio_dropped);

    // 3. Genuine overrun drops the NEW bytes, never corrupts what is queued.
    uart_out.clear(); radio_head=radio_tail=0; radio_dropped=0;
    uart_room=0;                                  // UART stalled
    std::string a(2000,'X');
    size_t took=hal_radio_write((const uint8_t*)a.data(),a.size());
    printf("  stalled UART: accepted %zu of %zu, dropped %u\n",took,a.size(),radio_dropped);
    expect("accepts only what the ring holds", took==sizeof(radio_tx)-1);
    expect("counts the loss", radio_dropped>0);
    uart_room=40;
    for(int i=0;i<200;i++) hal_radio_poll();
    expect("what was accepted comes out intact", uart_out==std::string(took,'X'));

    if(!fails) printf("ALL RADIO RING TESTS PASSED\n");
    return fails?1:0;
}
