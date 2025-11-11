module signal_adc_fsmc (
    input clk_20mhz,
    input START_FPGA,    
    output EMA_PULSE_P,
    output EMA_PULSE_N,
    output ON_32,
    output CTRL_SW,
    output CLK_P,               // Тактовый сигнал 80 МГц для АЦП
    input [11:0] adc_data,
    
    // === FSMC Interface ===
    input FPGA_OE,             // Сигнал чтения (активный 0)
    output [15:0] FSMC_D       // Шина данных
);

// ================== PLL для генерации 80 МГц ==================
wire clk_80mhz;
wire pll_locked;

pll_20_to_80 pll_inst (
    .inclk0(clk_20mhz),
    .c0(clk_80mhz),
    .locked(pll_locked)
);

assign CLK_P = clk_80mhz;

// ================== Синхронизация START_FPGA ==================
reg [1:0] start_sync;
always @(posedge clk_80mhz) begin
    start_sync <= {start_sync[0], START_FPGA};
end
wire start_pulse = (start_sync == 2'b01);

// ================== Signal Generator ==================
reg [23:0] counter = 0;
reg measurement_active = 0;

// Основной счетчик - запускается по START
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        counter <= 24'd0;
        measurement_active <= 1'b0;
    end else if (start_pulse) begin
        counter <= 24'd0;
        measurement_active <= 1'b1;
    end else if (measurement_active) begin
        if (counter < 24'hFFFFFF) begin
            counter <= counter + 24'd1;
        end else begin
            counter <= 24'd0;
            measurement_active <= 1'b0; // Завершение измерения
        end
    end
end

// Генерация сигнала EMA_PULSE_P
reg ema_pulse_p_reg = 1;
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        ema_pulse_p_reg <= 1;
    end else if (measurement_active) begin
        case (counter)
            24'd6400000: ema_pulse_p_reg <= 0;
            24'd6400010: ema_pulse_p_reg <= 1;
            24'd6400023: ema_pulse_p_reg <= 0;
            24'd6400033: ema_pulse_p_reg <= 1;
            default: ; // Сохраняем текущее значение
        endcase
    end else begin
        ema_pulse_p_reg <= 1;
    end
end
assign EMA_PULSE_P = ema_pulse_p_reg;

// Генерация сигнала EMA_PULSE_N
reg ema_pulse_n_reg = 1;
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        ema_pulse_n_reg <= 1;
    end else if (measurement_active) begin
        case (counter)
            24'd6400010: ema_pulse_n_reg <= 0;
            24'd6400020: ema_pulse_n_reg <= 1;
            24'd6400033: ema_pulse_n_reg <= 0;
            24'd6400043: ema_pulse_n_reg <= 1;
            default: ; // Сохраняем текущее значение
        endcase
    end else begin
        ema_pulse_n_reg <= 1;
    end
end
assign EMA_PULSE_N = ema_pulse_n_reg;

// Генерация сигнала ON_32
reg on_32_reg = 1;
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        on_32_reg <= 1;
    end else if (measurement_active) begin
        if (counter < 24'd1) begin
            on_32_reg <= 1;
        end else if (counter < 24'd6400043) begin
            on_32_reg <= 0;
        end else begin
            on_32_reg <= 1;
        end
    end else begin
        on_32_reg <= 1;
    end
end
assign ON_32 = on_32_reg;

// Генерация сигнала CTRL_SW
reg ctrl_sw_reg = 0;
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        ctrl_sw_reg <= 0;
    end else if (measurement_active) begin
        if (counter < 24'd6399999) begin
            ctrl_sw_reg <= 0;
        end else if (counter < 24'd6400203) begin
            ctrl_sw_reg <= 1;
        end else begin
            ctrl_sw_reg <= 0;
        end
    end else begin
        ctrl_sw_reg <= 0;
    end
end
assign CTRL_SW = ctrl_sw_reg;

// ================== Захват данных с АЦП ==================
// Таймер старта захвата данных (через N тактов после START_FPGA)
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

// Буфер на 10,000 значений
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

// ================== FSMC Interface ==================
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

endmodule