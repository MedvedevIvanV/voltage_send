module signal_adc_fsmc (
    input clk_20mhz,
    input START_FPGA,    
    output ON_32,               // Импульс 200 нс (отдельный сигнал, не связан с захватом)
    output CLK_P,               // Тактовый сигнал 80 МГц для АЦП
    input [11:0] adc_data,
    
    // === FSMC Interface ===
    input FPGA_OE,             // Сигнал чтения (активный 0)
    output [15:0] FSMC_D,      // Шина данных
    
    // === Отладочные сигналы ===
    output PER                 // Светодиод: "Буфер переполнен" (1 = ошибка)
);

// --- PLL 20->80 МГц ---
wire clk_80mhz;
wire pll_locked;

pll_20_to_80 pll_inst (
    .inclk0(clk_20mhz),
    .c0(clk_80mhz),
    .locked(pll_locked)
);

assign CLK_P = clk_80mhz;

// --- Синхронизация START_FPGA ---
reg [1:0] start_sync;
always @(posedge clk_80mhz) begin
    start_sync <= {start_sync[0], START_FPGA};
end
wire start_pulse = (start_sync == 2'b01);

// --- Генератор ON_32 (200 нс импульс через 2000 нс после старта) ---
reg [7:0] on32_delay_cnt;
reg on32_pulse;
reg [3:0] on32_width_cnt;

always @(posedge clk_80mhz) begin
    if (start_pulse) begin
        on32_delay_cnt <= 0;
        on32_pulse <= 0;
    end 
    else if (!on32_pulse && on32_delay_cnt < 160) begin  // 2000 нс задержка (160 тактов @ 80 МГц)
        on32_delay_cnt <= on32_delay_cnt + 1;
    end
    else if (!on32_pulse) begin
        on32_pulse <= 1;  // Запуск импульса
        on32_width_cnt <= 0;
    end
    else if (on32_width_cnt < 15) begin  // 200 нс длительность (16 тактов)
        on32_width_cnt <= on32_width_cnt + 1;
    end
    else begin
        on32_pulse <= 0;  // Импульс завершен
    end
end

assign ON_32 = on32_pulse;

// --- Таймер старта захвата данных (через N тактов после START_FPGA) ---
reg [15:0] start_capture_delay = 0;
reg capture_enable = 0;

// Задержка перед захватом (например, 5000 тактов = 62.5 мкс @ 80 МГц)
always @(posedge clk_80mhz) begin
    if (start_pulse) begin
        start_capture_delay <= 0;
        capture_enable <= 0;
    end 
    else if (!capture_enable && start_capture_delay < 5000) begin
        start_capture_delay <= start_capture_delay + 1;
    end
    else if (!capture_enable) begin
        capture_enable <= 1;  // Разрешение захвата
    end
end

// --- Буфер на 10,000 значений и обработка переполнения ---
reg [11:0] adc_buffer [0:9999];
reg [15:0] sample_counter = 0;
reg capture_done = 0;
reg buffer_overflow = 0;  // Флаг переполнения

always @(posedge clk_80mhz) begin
    if (start_pulse) begin
        sample_counter <= 0;
        capture_done <= 0;
        buffer_overflow <= 0;
    end
    else if (capture_enable && !capture_done) begin
        if (sample_counter < 10000) begin
            adc_buffer[sample_counter] <= adc_data;
            sample_counter <= sample_counter + 1;
        end
        else begin
            buffer_overflow <= 1;  // Буфер переполнен!
            capture_done <= 1;
        end
    end
end

// --- FSMC Interface ---
reg [15:0] read_ptr = 0;

always @(posedge clk_80mhz) begin
    if (start_pulse) begin
        read_ptr <= 0;
    end
    else if (capture_done && !FPGA_OE && read_ptr < 10000) begin
        read_ptr <= read_ptr + 1;
    end
end

assign FSMC_D = (!FPGA_OE && read_ptr < 10000) ? {4'b0000, adc_buffer[read_ptr]} : 16'h0000;

// --- Отладочный сигнал PER ---
assign PER = buffer_overflow;  // Светодиод загорится при переполнении

endmodule