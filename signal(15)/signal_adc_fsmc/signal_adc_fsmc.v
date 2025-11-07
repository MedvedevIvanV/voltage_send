module signal_adc_fsmc (
    input clk_20mhz,
    output EMA_PULSE_N,
    output EMA_PULSE_P,
    output ON_32,
    output CTRL_SW
);

// ================== PLL для генерации 80 МГц ==================
wire clk_80mhz;
wire pll_locked;

pll_20_to_80 pll_inst (
    .inclk0(clk_20mhz),
    .c0(clk_80mhz),
    .locked(pll_locked)
);

// ================== Signal Generator ==================
reg [23:0] counter = 0;
reg measurement_active = 1'b1; // Всегда активно (бесконечный цикл)

// Основной счетчик
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        counter <= 24'd0;
    end else begin
        if (counter < 24'hFFFFFF) begin
            counter <= counter + 24'd1;
        end else begin
            counter <= 24'd0; // Сброс счетчика для бесконечного цикла
        end
    end
end

// Генерация сигнала EMA_PULSE_P
reg ema_pulse_p_reg = 1;
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        ema_pulse_p_reg <= 1;
    end else begin
        case (counter)
            24'd6400000: ema_pulse_p_reg <= 0;
            24'd6400010: ema_pulse_p_reg <= 1;
            24'd6400023: ema_pulse_p_reg <= 0;
            24'd6400033: ema_pulse_p_reg <= 1;
            default: ; // Сохраняем текущее значение
        endcase
    end
end
assign EMA_PULSE_P = ema_pulse_p_reg;

// Генерация сигнала EMA_PULSE_N
reg ema_pulse_n_reg = 1;
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        ema_pulse_n_reg <= 1;
    end else begin
        case (counter)
            24'd6400010: ema_pulse_n_reg <= 0;
            24'd6400020: ema_pulse_n_reg <= 1;
            24'd6400033: ema_pulse_n_reg <= 0;
            24'd6400043: ema_pulse_n_reg <= 1;
            default: ; // Сохраняем текущее значение
        endcase
    end
end
assign EMA_PULSE_N = ema_pulse_n_reg;

// Генерация сигнала ON_32
reg on_32_reg = 1;
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        on_32_reg <= 1;
    end else begin
        if (counter < 24'd1) begin
            on_32_reg <= 1;
        end else if (counter < 24'd6400043) begin
            on_32_reg <= 0;
        end else begin
            on_32_reg <= 1;
        end
    end
end
assign ON_32 = on_32_reg;

// Генерация сигнала CTRL_SW
reg ctrl_sw_reg = 0;
always @(posedge clk_80mhz) begin
    if (!pll_locked) begin
        ctrl_sw_reg <= 0;
    end else begin
        if (counter < 24'd6399999) begin
            ctrl_sw_reg <= 0;
        end else if (counter < 24'd6400203) begin
            ctrl_sw_reg <= 1;
        end else begin
            ctrl_sw_reg <= 0;
        end
    end
end
assign CTRL_SW = ctrl_sw_reg;

endmodule