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
# Базовые тактовые сигналы
create_clock -name clk_20mhz -period 50.000 [get_ports clk_20mhz]

# Автоматическое определение тактов PLL
derive_pll_clocks
derive_clock_uncertainty

# Группировка тактовых доменов
set_clock_groups -asynchronous -group {clk_20mhz}
set_clock_groups -asynchronous -group [get_clocks {*|pll_20_to_80|*clk[0]}]

# Специфичные ограничения для интерфейса FSMC
set_input_delay -clock [get_clocks {clk_80mhz}] -max 5.000 [get_ports {FPGA_OE}]
set_output_delay -clock [get_clocks {clk_80mhz}] -max 3.000 [get_ports {FSMC_D[*]}]

# Критический путь: синхронизация START_FPGA
set_false_path -from [get_ports {START_FPGA}] -to [get_registers {start_sync[*]}]

# Ограничения для выхода ON_32
set_output_delay -clock [get_clocks {clk_80mhz}] -max 1.000 [get_ports {ON_32}]
set_max_delay -from [get_registers {pulse_active}] -to [get_ports {ON_32}] 2.000
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

