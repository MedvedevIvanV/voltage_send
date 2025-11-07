## Generated SDC file "signal_adc_fsmc.sdc"

## Copyright (C) 2021  Intel Corporation. All rights reserved.
## Your use of Intel Corporation's design tools, logic functions 
## and other software and tools, and any partner logic 
## functions, and any output files from any of the foregoing 
## (including device programming or simulation files), and any 
## associated documentation or information are expressly subject 
## to the terms and conditions of the Intel Program License 
## Subscription Agreement, the Intel Quartus Prime License Agreement,
## the Intel FPGA IP License Agreement, or other applicable license
## agreement, including, without limitation, that your use is for
## the sole purpose of programming logic devices manufactured by
## Intel and sold by Intel or its authorized distributors.  Please
## refer to the applicable agreement for further details, at
## https://fpgasoftware.intel.com/eula.


## VENDOR  "Altera"
## PROGRAM "Quartus Prime"
## VERSION "Version 21.1.0 Build 842 10/21/2021 SJ Lite Edition"

## DATE    "Thu May 29 11:04:10 2025"

##
## DEVICE  "10CL006YE144A7G"
##


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3



#**************************************************************
# Create Clock
#**************************************************************

# ================= Тактовые сигналы =================
# Основной входной такт 20 МГц
create_clock -name clk_20mhz -period 50.000 [get_ports clk_20mhz]

# Такт 80 МГц от PLL
derive_pll_clocks -create_base_clocks
derive_clock_uncertainty

# ================= Выходные ограничения =================
# Все выходные сигналы должны укладываться в 8ns (соответствует 80 МГц)
set_output_delay -clock [get_clocks {clk_80mhz}] -max 8.000 [all_outputs]

# ================= Входные ограничения =================
set_input_delay -clock [get_clocks {clk_20mhz}] -max 5.000 [get_ports clk_20mhz]

# ================= Особые ограничения для сигналов =================
# Для импульсных сигналов EMA устанавливаем более строгие требования
set_output_delay -clock [get_clocks {clk_80mhz}] -max 5.000 \
    [get_ports {EMA_PULSE_N EMA_PULSE_P}]

# Для медленных сигналов можно ослабить требования
set_output_delay -clock [get_clocks {clk_80mhz}] -max 10.000 \
    [get_ports {ON_32 CTRL_SW}]

# ================= Глобальные ограничения =================
# Исправленная команда:
set_clock_uncertainty -setup -to [all_clocks] 0.500
set_clock_latency -source 1.000 [get_clocks {clk_20mhz}]

# ================= Исключения =================
# Исключаем из анализа пути счётчика (он может быть длинным)
set_false_path -from [get_registers {cnt*}] -to [get_registers {cnt*}]

# ================= Глобальные ограничения =================
set_clock_uncertainty -setup -to [all_clocks] 0.500
set_clock_latency -source 1.000 [get_clocks clk_20mhz]

# ================= Исключения =================
# Получаем полное имя регистра счетчика
set cnt_regs [get_registers -nowarn {*cnt*|*counter*}]

if {[llength $cnt_regs] > 0} {
    set_false_path -from $cnt_regs -to $cnt_regs
} else {
    post_message -type warning "Не найдены регистры счетчика для исключения"
}

# ================= Дополнительные настройки =================
# Устанавливаем стандартные требования для всех путей
set_max_delay -from [all_clocks] -to [all_outputs] 10.000

#**************************************************************
# Create Generated Clock
#**************************************************************



#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************

set_clock_uncertainty -rise_from [get_clocks {clk_20mhz}] -rise_to [get_clocks {clk_20mhz}] -setup 0.200  
set_clock_uncertainty -rise_from [get_clocks {clk_20mhz}] -rise_to [get_clocks {clk_20mhz}] -hold 0.020  
set_clock_uncertainty -rise_from [get_clocks {clk_20mhz}] -fall_to [get_clocks {clk_20mhz}] -setup 0.200  
set_clock_uncertainty -rise_from [get_clocks {clk_20mhz}] -fall_to [get_clocks {clk_20mhz}] -hold 0.020  
set_clock_uncertainty -fall_from [get_clocks {clk_20mhz}] -rise_to [get_clocks {clk_20mhz}] -setup 0.200  
set_clock_uncertainty -fall_from [get_clocks {clk_20mhz}] -rise_to [get_clocks {clk_20mhz}] -hold 0.020  
set_clock_uncertainty -fall_from [get_clocks {clk_20mhz}] -fall_to [get_clocks {clk_20mhz}] -setup 0.200  
set_clock_uncertainty -fall_from [get_clocks {clk_20mhz}] -fall_to [get_clocks {clk_20mhz}] -hold 0.020  


#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************



#**************************************************************
# Set False Path
#**************************************************************




#**************************************************************
# Set Multicycle Path
#**************************************************************



#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************

