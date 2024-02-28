#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "st7735_lcd_lib.h"
#include "st77xx_lcd.pio.h"

PIO pio_st7735;
uint sm_st7735;
uint dma_chan_st7735;
uint offset_st7735;
pio_sm_config c_st7735;
// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little
static const uint8_t st7735_init_seq[] = {
        1, 30, ST7735_SWRESET,                        // Software reset
        1, 100, ST7735_SLPOUT,                        // Exit sleep mode
        4, 2, ST7735_FRMCTR1, 5,20,20,//0x01, 0x2C, 0x2D,      //  11111=4: Frame rate control, fastest refresh, 6 lines front porch, 3 lines back porch
        4, 2, ST7735_FRMCTR2, 5,20,30,//0x01, 0x2C, 0x2D,      //  11111=4: Frame rate control, fastest refresh, 6 lines front porch, 3 lines back porch
        7, 2, ST7735_FRMCTR3, 5,20,30,5,20,30,      //  11111=4: Frame rate control, fastest refresh, 6 lines front porch, 3 lines back porch
        2, 0, ST7735_INVCTR, 0x07,                   //6: Display inversion ctrl// Set MADCTL: row then column, refresh is bottom to top ????
        // 3, 0, 0xB6, 0x15, 0x02,             // 6: Display settings
        // 2, 0, 0xB4, 0x0,                   // 7: Display inversion control
        4, 0, ST7735_PWCTR1, 0xA2, 0x02, 0x84,            // 8: Power control
        2, 0, ST7735_PWCTR2, 0xC5,                   // 9: Power control
        3, 0, ST7735_PWCTR3, 0x0A, 0x00,             // 10: Power control
        3, 0, ST7735_PWCTR4, 0x8A, 0x2A,             // 10: Power control
        3, 0, ST7735_PWCTR5, 0x8A, 0xEE,             // 11: Power control
        2, 0, ST7735_VMCTR1, 0x0E,            // 12: Power control
        1, 0, ST7735_INVOFF,                 // 13: Don't invert display

        2, 0, ST7735_MADCTL, ST7735_ROTATION,                   // Set MADCTL: row then column, refresh is bottom to top ????
        2, 2, ST7735_COLMOD, 0x05,                   // Set colour mode to 16 bit
        
        //3, 0, 0xFC, 0x11, 0x15,              // 12: Power control
        
        5, 0, ST7735_CASET, 0x00, 0x00, 0x00, 127,   // CASET: column addresses
        5, 0, ST7735_RASET, 0x00, 0x00, 0x00, 0xA0, // RASET: row addresses
        
        
        17, 2,  ST7735_GMCTRP1, 0x02, 0x1c, 0x07, 0x12, // 13: Gamma Adjustments
                0x37, 0x32, 0x29, 0x2d,
                0x29, 0x25, 0x2B, 0x39,
                0x00, 0x01, 0x03, 0x10,
        17, 2,  ST7735_GMCTRN1, 0x03, 0x1d, 0x07, 0x06, // 14: Gamma Adjustments
                0x2E, 0x2C, 0x29, 0x2D,
                0x2E, 0x2E, 0x37, 0x3F,
                0x00, 0x00, 0x02, 0x10,      
            
        //1, 2, 0x21,                         // Inversion on, then 10 ms delay (supposedly a hack?)
        1, 2, ST7735_NORON,                         // Normal display on, then 10 ms delay
        1, 20, ST7735_DISPON,                         // Main screen turn on, then wait 500 ms
        0                                   // Terminate list
};

// set OSR autopull threshold to n_bits
// n_bits should be 0 ~ 32
// static inline void sm_set_out_autopull_threshold(uint8_t n_bits){
//     static uint32_t shiftctrl_reg_temp;
//     // Halt the machine, set some sensible defaults
//     pio_sm_set_enabled(pio_st7735, sm_st7735, false);

//     // set out shift autopull threshold to n_bits bits
//     shiftctrl_reg_temp=pio_st7735->sm[sm_st7735].shiftctrl;
//     shiftctrl_reg_temp&=0xe0ffffff; //clear pull threshoud bits [29:25]
//     shiftctrl_reg_temp|=n_bits<<25; //set pull threshoud bits [29:25]
//     pio_st7735->sm[sm_st7735].shiftctrl=shiftctrl_reg_temp; 

//     pio_sm_clear_fifos(pio_st7735, sm_st7735);

//     // Clear FIFO debug flags
//     const uint32_t fdebug_sm_mask =
//             (1u << PIO_FDEBUG_TXOVER_LSB) |
//             (1u << PIO_FDEBUG_RXUNDER_LSB) |
//             (1u << PIO_FDEBUG_TXSTALL_LSB) |
//             (1u << PIO_FDEBUG_RXSTALL_LSB);
//     pio_st7735->fdebug = fdebug_sm_mask << sm_st7735;

//     // Finally, clear some internal SM state
//     pio_sm_restart(pio_st7735, sm_st7735);
//     pio_sm_clkdiv_restart(pio_st7735, sm_st7735);
//     pio_sm_exec(pio_st7735, sm_st7735, pio_encode_jmp(offset_st7735));
//     pio_sm_set_enabled(pio_st7735, sm_st7735, true);
// }

static inline void sm_config_init(){
    c_st7735 = st77xx_lcd_program_get_default_config(offset_st7735);
    sm_config_set_sideset_pins(&c_st7735, ST7735_PIN_CLK);
    sm_config_set_out_pins(&c_st7735, ST7735_PIN_DIN, 1);
    sm_config_set_fifo_join(&c_st7735, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c_st7735, ST7735_SERIAL_CLK_DIV);
    sm_config_set_out_shift(&c_st7735, false, true, 8);
}
static inline void sm_set_out_autopull_threshold(uint8_t n_bits){
    sm_config_set_out_shift(&c_st7735, false, true, n_bits);
    pio_sm_init(pio_st7735, sm_st7735, offset_st7735, &c_st7735);
    pio_sm_set_enabled(pio_st7735, sm_st7735, true);
}


static inline void lcd_set_dc_cs(bool dc, bool cs) {
    sleep_us(1);
    gpio_put_masked((1u << ST7735_PIN_DC) | (1u << ST7735_PIN_CS), !!dc << ST7735_PIN_DC | !!cs << ST7735_PIN_CS);
    sleep_us(1);
}

static inline void lcd_write_cmd(const uint8_t *cmd, size_t count) {
    st77xx_lcd_wait_idle(pio_st7735, sm_st7735);
    lcd_set_dc_cs(0, 0);
    st77xx_lcd_put(pio_st7735, sm_st7735, *cmd++);
    if (count >= 2) {
        st77xx_lcd_wait_idle(pio_st7735, sm_st7735);
        lcd_set_dc_cs(1, 0);
        for (size_t i = 0; i < count - 1; ++i)
            st77xx_lcd_put(pio_st7735, sm_st7735, *cmd++);
    }
    st77xx_lcd_wait_idle(pio_st7735, sm_st7735);
    lcd_set_dc_cs(1, 1);
}

static inline void lcd_init(const uint8_t *init_seq) {
    const uint8_t *cmd = init_seq;
    while (*cmd) {
        lcd_write_cmd(cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }
}

static inline void st7735_start_pixels(void) {
    uint8_t cmd = ST7735_RAMWR; // RAMWR
    lcd_write_cmd(&cmd, 1);
    lcd_set_dc_cs(1, 0);
}

// dma传送数据函数
// 128*160 = 20,480 halfwords
// databits = 20,480 * 16 = 327,680 bits
// full refresh time = 327,680 / 30MHz = 10.92267 ms
static inline void st77xx_dma_transfer_packet(uint16_t *capture_buf, size_t capture_size_halfwords, bool is_read_inc) 
{
    dma_channel_config c = dma_channel_get_default_config(dma_chan_st7735);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, is_read_inc);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio_st7735, sm_st7735, true));

    dma_channel_configure(dma_chan_st7735, &c,
        &pio_st7735->txf[sm_st7735],        // Destination pointer
        capture_buf,         // Source pointer
        capture_size_halfwords, // Number of transfers
        true                // Start immediately
    );
}

// 初始化函数，传入参数为pio，sm和dma_chan自动申请
void st7735_lcd_init(PIO pio){
    pio_st7735 = pio;
    sm_st7735 = pio_claim_unused_sm(pio_st7735, true);
    dma_chan_st7735 = dma_claim_unused_channel(true);;
    offset_st7735 = pio_add_program(pio_st7735, &st77xx_lcd_program);
    st77xx_lcd_program_init(pio_st7735, sm_st7735, offset_st7735, ST7735_PIN_DIN, ST7735_PIN_CLK, ST7735_SERIAL_CLK_DIV, 8);
    sm_config_init();
    gpio_init(ST7735_PIN_CS);
    gpio_init(ST7735_PIN_DC);
    gpio_init(ST7735_PIN_RESET);
    gpio_init(ST7735_PIN_BL);
    gpio_set_dir(ST7735_PIN_CS, GPIO_OUT);
    gpio_set_dir(ST7735_PIN_DC, GPIO_OUT);
    gpio_set_dir(ST7735_PIN_RESET, GPIO_OUT);
    gpio_set_dir(ST7735_PIN_BL, GPIO_OUT);

    gpio_put(ST7735_PIN_CS, 1);
    gpio_put(ST7735_PIN_RESET, 1);
    lcd_init(st7735_init_seq);
    gpio_put(ST7735_PIN_BL, 1);
}

static inline void st7735_set_address_window_start_pixels(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    
    // set out shift autopull threshold to 8 bits
    sm_set_out_autopull_threshold(8);
    //st77xx_lcd_program_init(pio_st7735, sm_st7735, offset, ST7735_PIN_DIN, ST7735_PIN_CLK, ST7735_SERIAL_CLK_DIV, 8);
    // column address set
    uint8_t data[] = {ST7735_CASET, 0x00, x0 + ST7735_XSTART, 0x00, x1 + ST7735_XSTART};
    lcd_write_cmd(data, sizeof(data)/sizeof(uint8_t) );

    // row address set
    data[0] = ST7735_RASET;
    data[2] = y0 + ST7735_YSTART;
    data[4] = y1 + ST7735_YSTART;
    lcd_write_cmd(data, sizeof(data)/sizeof(uint8_t));

    // write to RAM
    data[0] = ST7735_RAMWR;
    lcd_write_cmd(data, 1);

    // set out shift autopull threshold to 16 bits
    sm_set_out_autopull_threshold(16);
    //st77xx_lcd_program_init(pio_st7735, sm_st7735, offset, ST7735_PIN_DIN, ST7735_PIN_CLK, ST7735_SERIAL_CLK_DIV, 16);
    
    lcd_set_dc_cs(1, 0);
}

inline void st7735_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
        return;
    st7735_set_address_window_start_pixels(x, y, x+1, y+1);
    st77xx_lcd_wait_idle(pio_st7735, sm_st7735);
    st77xx_lcd_put16(pio_st7735, sm_st7735, color);
}

static inline void st7735_write_char(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
    static uint32_t i, b, j;

    st7735_set_address_window_start_pixels(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                st77xx_lcd_wait_idle(pio_st7735, sm_st7735);
                st77xx_lcd_put16(pio_st7735, sm_st7735, color);
            } else {
                st77xx_lcd_wait_idle(pio_st7735, sm_st7735);
                st77xx_lcd_put16(pio_st7735, sm_st7735, bgcolor);
            }
        }
    }
}

void st7735_write_string(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
    while(*str) {
        if(x + font.width >= ST7735_WIDTH) {
            x = 0;
            y += font.height;
            if(y + font.height >= ST7735_HEIGHT) {
                break;
            }

            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }
        st7735_write_char(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }
}

void st7735_fill_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // clipping
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
    if((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    st7735_set_address_window_start_pixels(x, y, x+w-1, y+h-1);
    //st7735_start_pixels();
    for(y = h; y > 0; y--) {
        for(x = w; x > 0; x--) {
            st77xx_lcd_wait_idle(pio_st7735, sm_st7735);
            st77xx_lcd_put16(pio_st7735, sm_st7735, color);
        }
    }
}

void st7735_fill_rectangle_dma(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // clipping
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
    if((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    st7735_set_address_window_start_pixels(x, y, x+w-1, y+h-1);
    st77xx_dma_transfer_packet(&color, w*h, false);
}

void st7735_fill_screen(uint16_t color) {
    st7735_fill_rectangle(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void st7735_fill_screen_dma(uint16_t color) {
    st7735_fill_rectangle_dma(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void st7735_draw_image_dma(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) return;
    if((y + h - 1) >= ST7735_HEIGHT) return;

    st7735_set_address_window_start_pixels(x, y, x+w-1, y+h-1);
    st77xx_dma_transfer_packet((uint16_t*)data, w*h, true);
}

void st7735_wait_dma_idel(void){
    dma_channel_wait_for_finish_blocking(dma_chan_st7735);
}
