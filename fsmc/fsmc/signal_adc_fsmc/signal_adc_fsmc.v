module signal_adc_fsmc (
    input clk_20mhz,
    output CLK_P,
    output EMA_PULSE_N,
    output EMA_PULSE_P,
    output ON_32,
    output CTRL_SW,
    
    // === FSMC Interface ===
    input FGPA_OE,       // FSMC_NOE (активный низкий)
    input FGPA_WE,       // FSMC_NWE (активный низкий)
    inout [15:0] FSMC_D,
    input FGPA_CLE,      // Не используется
    input FGPA_ALE       // Не используется
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

// ================== FSMC Interface ==================
reg [15:0] fsmc_out;
reg fsmc_dir; // Направление: 1 - ПЛИС -> STM, 0 - STM -> ПЛИС

// Трехстабиный буфер для шины данных
assign FSMC_D = (!FGPA_OE && fsmc_dir) ? fsmc_out : 16'hZZZZ;

// Данные для передачи (10000 значений: 0-4095, 4095-0, остальные 100)
reg [11:0] data_values [0:9999];
reg [13:0] data_index = 0; // Увеличиваем размер индекса для 10000 значений
reg data_ready = 0; // Флаг готовности данных

// Регистр для детектирования фронта OE
reg oe_prev = 1;

// Инициализация массива данных
integer i; // Объявляем переменную цикла вне блока initial
initial begin
    // Первая часть: от 0 до 4095 (4096 значений)
    for (i = 0; i < 4096; i = i + 1) begin
        data_values[i] = i;
    end
    
    // Вторая часть: от 4095 до 0 (4096 значений)
    for (i = 4096; i < 8192; i = i + 1) begin
        data_values[i] = 4095 - (i - 4096);
    end
    
    // Третья часть: оставшиеся 1808 значений = 100
    for (i = 8192; i < 10000; i = i + 1) begin
        data_values[i] = 100;
    end
end

always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        // Сброс всех регистров при отсутствии PLL
        data_index <= 0;
        fsmc_dir <= 0;
        fsmc_out <= 0;
        data_ready <= 0;
        oe_prev <= 1;
    end else begin
        // Детектируем фронт сигнала FGPA_OE (переход 1->0)
        oe_prev <= FGPA_OE;
        
        // Основная логика передачи данных
        if (!FGPA_OE) begin
            // Устанавливаем данные на шину
            fsmc_out <= {4'b0000, data_values[data_index]};
            fsmc_dir <= 1;
            
            // Переключаем индекс по фронту OE
            if (oe_prev && !FGPA_OE) begin // Обнаружен фронт
                if (data_ready) begin
                    if (data_index < 9999) begin
                        data_index <= data_index + 1;
                    end else begin
                        data_index <= 0;
                        data_ready <= 0; // Все данные переданы
                    end
                end
            end
        end else begin
            fsmc_dir <= 0;
        end
        
        // Автоматическая подготовка новых данных каждые 3 секунды
        if (data_ready == 0) begin
            // Здесь можно добавить логику генерации новых данных
            // Пока просто устанавливаем флаг готовности
            data_ready <= 1;
            data_index <= 0;
        end
    end
end

// Неиспользуемые выходы
assign EMA_PULSE_N = 1'b1;
assign EMA_PULSE_P = 1'b1;
assign ON_32 = 1'b1;
assign CTRL_SW = 1'b0;

endmodule