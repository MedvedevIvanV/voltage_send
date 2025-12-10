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
    if (!pll_locked) begin
        start_sync <= 2'b00;
    end else begin
        start_sync <= {start_sync[0], START_FPGA};
    end
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
            default: ema_pulse_p_reg <= ema_pulse_p_reg; // Явное указание сохранения значения
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
            default: ema_pulse_n_reg <= ema_pulse_n_reg; // Явное указание сохранения значения
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

// Генерация сигнала CTRL_SW - постоянно 0
reg ctrl_sw_reg = 0;
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        ctrl_sw_reg <= 0;
    end else begin
        ctrl_sw_reg <= 0; // Всегда устанавливаем в 0
    end
end
assign CTRL_SW = ctrl_sw_reg;


// ================== Захват данных с АЦП ==================
// Буфер на 15,000 значений (исправлено с 10,000 на 15,000 согласно коду)
reg [11:0] adc_buffer [0:14999];
reg [13:0] sample_counter = 0; // 14 бит достаточно для 15000
reg capture_active = 0;
reg capture_done = 0;

// Флаг запуска захвата по достижении 6,400,000 тактов
reg start_capture_flag = 0;

// Управление запуском захвата данных (через 6,400,000 тактов)
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        start_capture_flag <= 0;
    end else if (measurement_active && counter == 24'd6400000) begin
        start_capture_flag <= 1; // Устанавливаем флаг запуска захвата
    end else begin
        start_capture_flag <= 0; // Сбрасываем флаг после одного такта
    end
end

// Управление захватом данных
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        sample_counter <= 0;
        capture_active <= 0;
        capture_done <= 0;
    end else if (start_capture_flag) begin
        sample_counter <= 0;
        capture_active <= 1; // Начинаем захват при counter = 6,400,000
        capture_done <= 0;
    end else if (capture_active) begin
        if (sample_counter < 15000) begin
            adc_buffer[sample_counter] <= adc_data;
            sample_counter <= sample_counter + 1;
        end else begin
            capture_active <= 0;
            capture_done <= 1; // Захват завершен
        end
    end
end

// ================== FSMC Interface ==================
reg [13:0] read_ptr = 0; // 14 бит для адресации 15000 значений
reg [15:0] fsmc_data_out;

// Управление указателем чтения
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        read_ptr <= 0;
    end else if (start_pulse) begin
        read_ptr <= 0;
    end else if (!FPGA_OE && capture_done) begin
        if (read_ptr < 14999) begin
            read_ptr <= read_ptr + 1;
        end
        // Не сбрасываем read_ptr, чтобы можно было читать данные многократно
    end
end

// Чтение из буфера
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        fsmc_data_out <= 16'h0000;
    end else if (!FPGA_OE && capture_done && read_ptr < 15000) begin
        fsmc_data_out <= {4'b0000, adc_buffer[read_ptr]};
    end else begin
        fsmc_data_out <= 16'h0000;
    end
end

assign FSMC_D = fsmc_data_out;

// ================== Debug Signals ==================
// Можно добавить для отладки
wire [13:0] debug_sample_count = sample_counter;
wire debug_capture_active = capture_active;
wire debug_capture_done = capture_done;

endmodule