module pin_69
(
    input wire CLK_0,      		// pin 22
    output wire EMA_PULSE_P,		// pin 68
	 output wire EMA_PULSE_N		// pin 69
);

// Тактовая частота 20МГц  с внешнего генератора пступает на 22 вывод МС
wire clk = CLK_0;

// локальны параметр задает количество разрядов двоичного счетчика - делителя
localparam COUNTER_LENGTH = 24;

// Двоичный счетчик разрядностью COUNTER_LENGTH 
reg [(COUNTER_LENGTH - 1) : 0] led_counter = 0;



always @(posedge clk) begin
	
	led_counter = led_counter + 1'b1;
end

// старший разряд счетчика инвертируется каждые 2^^(COUNTER_LENGTH - 2) тактов тактовой частоты 
// Что для тактовой частоты 20МГц должно составлять около 1с
assign EMA_PULSE_P = led_counter[COUNTER_LENGTH - 1];

// предшествующий старшему разряду счетчика инвертируется каждые 2^^(COUNTER_LENGTH - 3) тактов тактовой частоты 
// Что для тактовой частоты 20МГц должно составлять около 0,5с
assign EMA_PULSE_N = led_counter[COUNTER_LENGTH - 2];


endmodule