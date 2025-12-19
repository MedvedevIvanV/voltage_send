module signal_adc_fsmc (
    input clk_20mhz,
    input START_FPGA,    
    output EMA_PULSE_P,
    output EMA_PULSE_N,
    output ON_32,
    output CTRL_SW,
    output CLK_P,               // Тактовый сигнал 80 МГц для АЦП
    input [11:0] adc_data,      // Оставляем вход, но не используем
    
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

// Генерация сигнала CTRL_SW
//reg ctrl_sw_reg = 0;
//always @(posedge clk_80mhz) begin
//    if (!pll_locked) begin
//        ctrl_sw_reg <= 0;
//    end else begin
//        if (counter < 24'd6399999) begin
//            ctrl_sw_reg <= 0;
//        end else if (counter < 24'd6400203) begin
//            ctrl_sw_reg <= 1;
//        end else begin
//            ctrl_sw_reg <= 0;
//        end
//    end
//end
//assign CTRL_SW = ctrl_sw_reg;

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
reg [11:0] adc_buffer [0:4999];
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
        if (sample_counter < 5000) begin
            adc_buffer[sample_counter] <= adc_data;
            sample_counter <= sample_counter + 1;
        end else begin
            capture_active <= 0;
            capture_done <= 1; // Захват завершен
        end
    end
end


// Регистры для управления передачей данных в МК
reg [15:0] fsmc_data_out = 16'h0000;      // Выходные данные на FSMC
reg [12:0] read_index = 0;                // Индекс читаемого значения (0-4999)
reg [2:0] oe_sync = 3'b111;               // Синхронизатор сигнала FPGA_OE
reg prev_oe = 1'b1;                       // Предыдущее значение OE
reg read_active = 1'b0;                   // Флаг активного чтения
reg [1:0] ack_delay = 2'b00;              // Задержка для подтверждения
reg [12:0] transfer_counter = 0;          // Счетчик переданных значений

// Присваивание выходных данных
assign FSMC_D = fsmc_data_out;

// Синхронизация сигнала OE (активный 0)
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        oe_sync <= 3'b111;
        prev_oe <= 1'b1;
    end else begin
        oe_sync <= {oe_sync[1:0], FPGA_OE};  // Сдвиговый регистр для синхронизации
        prev_oe <= oe_sync[1];               // Запоминаем предыдущее значение
    end
end

// Определение фронтов сигнала OE
wire oe_falling_edge = (prev_oe == 1'b1 && oe_sync[1] == 1'b0);  // Передний фронт (начало чтения)
wire oe_rising_edge = (prev_oe == 1'b0 && oe_sync[1] == 1'b1);   // Задний фронт (конец чтения)

// Управление процессом передачи данных
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        // Сброс всех регистров при отсутствии блокировки PLL
        fsmc_data_out <= 16'h0000;
        read_index <= 0;
        read_active <= 1'b0;
        ack_delay <= 2'b00;
        transfer_counter <= 0;
    end else begin
        // Основная логика работы с МК
        if (capture_done) begin
            // Если захват данных завершен, выставляем бит 15 в 1
            // Это сигнализирует МК, что данные готовы к чтению
            fsmc_data_out[15] <= 1'b1;
        end
        
        // Обработка начала чтения по переднему фронту OE
        if (oe_falling_edge && capture_done) begin
            // МК начал запрос на чтение - сбрасываем бит 15 в 0
            fsmc_data_out[15] <= 1'b0;
            read_active <= 1'b1;          // Активируем процесс чтения
            ack_delay <= 2'b01;           // Устанавливаем задержку для подтверждения
        end
        
        // Обработка активного процесса чтения
        if (read_active) begin
            // Задержка для стабилизации и обработки сигналов
            if (ack_delay > 0) begin
                ack_delay <= ack_delay - 1;
            end else begin
                // После задержки выставляем бит 14 в 1 и данные
                // Это сигнализирует МК, что данные готовы
                fsmc_data_out <= {1'b0, 1'b1, 1'b0, adc_buffer[read_index]};  // [15]=0, [14]=1, [13]=0, [11:0]=данные
            end
            
            // Обработка завершения чтения (задний фронт OE)
            if (oe_rising_edge) begin
                // МК завершил чтение текущего значения
                // Готовимся к следующему запросу
                fsmc_data_out <= 16'h0000;    // Сбрасываем выходные данные
                
                if (read_index < 4999) begin
                    // Если еще не все данные переданы
                    read_index <= read_index + 1;  // Увеличиваем индекс
                    ack_delay <= 2'b01;           // Сбрасываем задержку для следующего чтения
                end else begin
                    // Все 5000 значений переданы
                    read_index <= 0;              // Сбрасываем индекс
                    read_active <= 1'b0;          // Деактивируем чтение
                    
                    // После передачи всех данных можно сбросить capture_done
                    // если нужно начать новый захват
                    // capture_done <= 1'b0;
                end
            end
        end else if (!capture_done) begin
            // Если захват не завершен, держим все выходы в 0
            fsmc_data_out <= 16'h0000;
        end
    end
end

endmodule
