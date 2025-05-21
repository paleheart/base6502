
module clock_divider (
    input wire clk_in,
    input wire reset,
    output reg clk_out
);
    reg [24:0] counter;

    always @(posedge clk_in or posedge reset) begin
        if (reset) begin
            counter <= 0;
            clk_out <= 0;
        end else begin
            counter <= counter + 1;
            if (counter == 3_500_000) begin // adjust for your clock rate
                counter <= 0;
                clk_out <= ~clk_out;
            end
        end
    end
endmodule



module cpu (
    //input wire clk,
    input wire reset,
    //output wire [15:0] address,
    //inout wire [7:0] data_in,
    //output wire write_enable,
    //output wire output_enable,
    output wire [5:0] led


    
);
    wire slower_clk;
    wire slow_clk;

    Gowin_OSC internal_clock (
        .oscout(slow_clk)
    );
    
    clock_divider div_inst (
        .clk_in(slow_clk),   // your internal oscillator
        .reset(reset),      // your reset signal
        .clk_out(slower_clk)
    );

initial begin
    ram[0]  = 8'hA9; ram[1]  = 8'h01;        // LDA #$01
    // loop:
    ram[2]  = 8'h0A;                         // ASL A
    ram[3]  = 8'hC9; ram[4]  = 8'h40;        // CMP #$40
    ram[5]  = 8'hD0; ram[6]  = 8'h02;        // BNE +2 (to loop_cont)
    ram[7]  = 8'hA9; ram[8]  = 8'h01;        // LDA #$01
    // loop_cont:
    ram[9]  = 8'h4C; ram[10] = 8'h02; ram[11] = 8'h00; // JMP $0002 (loop)
end




    // 2KB SRAM
    reg [7:0] ram [0:2047];

    // 256B ROM (adjust size as needed)
    reg [7:0] rom [0:255];
  
    //registers
    reg [15:0] pc;                          //16 bit - Program counter
    reg [7:0] ir;                           //8 bit - instruction register
    reg [1:0] fetch_decode;                 //2 bit - fetch decode state machine register
    reg [7:0] a;                            //8 bit - cpu accumulator register
    reg [3:0] decode_phase;                 //4 bit- current jump phase
    reg [7:0] addr_low;                      //8 bit - jump low byte
    reg [7:0] addr_high;                     //8 bit - jump high byte
    reg [7:0] x;                            //8 bit- x register
    reg [7:0] y;
    reg [15:0] return;                      //16 bit - return address
    reg [7:0] sp;                           //8 bit - stack pointer
    reg set_write_enable;
    reg [7:0] data_out;                     // Register for writing to the bus
    reg [7:0] data_in;
    reg [7:0] temp_math;
    reg [7:0] temp_result;
    reg carry_flag;
    reg zero_flag;
    reg overflow_flag;
    reg [7:0] branch_offset;
    reg negative_flag;
    reg no_advance;
    reg [7:0] and_result;
    reg [7:0] or_result;
    reg [5:0] led_reg;
    
    //assign address = pc;                    //Make the address lines match the current address in program counter
    //assign write_enable = set_write_enable;
    //assign output_enable = set_write_enable;
    //assign data_in = set_write_enable ? data_out : 8'bz;


    //Opcode definitions
    parameter opcode_NOP = 8'hEA;           //no operation
    parameter opcode_LDA_IMM = 8'hA9;       //load immediate into accumulator
    parameter opcode_LDA_ABS = 8'hAD;       //load memory address value into accumulator
    parameter opcode_LDA_ZER = 8'hA5;       //load zero page address into accumulator
    parameter opcode_JMP = 8'h4C;           //jump to address
    parameter opcode_INX = 8'hE8;           //increment x
    parameter opcode_INY = 8'hC8;           //increment y
    parameter opcode_LDX_IMM = 8'hA2;       //load x immediate
    parameter opcode_LDY_IMM = 8'hA0;
    parameter opcode_TAX = 8'hAA;           //transfer accumulator to x
    parameter opcode_TAY = 8'hA8;           //transfer accumulator to y
    parameter opcode_TXA = 8'h8A;           //transfer x to accumulator
    parameter opcode_TYA = 8'h98;           //transfer y to accumulator
    parameter opcode_DEX = 8'hCA;           //decrement x
    parameter opcode_DEY = 8'h88;           //decrement y
    parameter opcode_JSR = 8'h20;           //jump and store current location in stack
    parameter opcode_STA_ABS = 8'h8D;       //store accumulator in ram location
    parameter opcode_PHA = 8'h48;           //push accumulator to stack
    parameter opcode_PLA = 8'h68;           //pull stack to accumulator
    parameter opcode_RTS = 8'h60;           //read two byte address from stack and jmp
    parameter opcode_ADC_IMM = 8'h42;
    parameter opcode_CLC = 8'h18;
    parameter opcode_BEQ = 8'hF0;
    parameter opcode_BNE = 8'hd0;
    parameter opcode_CMP_IMM = 8'hC9;
    parameter opcode_SBC_IMM = 8'hE9;
    parameter opcode_SEC = 8'h38;
    parameter opcode_AND_IMM = 8'h29;
    parameter opcode_ORA_IMM = 8'h09;
    parameter opcode_EOR_IMM = 8'h49;
    parameter opcode_ASL = 8'h0A;
    parameter opcode_LDA_INX = 8'hA1;
    parameter opcode_STA_ZER = 8'h85;
    parameter opcode_LDA_INY = 8'hB1;
    parameter opcode_STA_INX = 8'h81;
    parameter opcode_LDX_ZER = 8'hA6;
    parameter opcode_LDY_ZER = 8'hA4;
    parameter opcode_LDX_ABS = 8'hAE;
    parameter opcode_LDY_ABS = 8'hAC;
    parameter opcode_STX_ZER = 8'h86;
    parameter opcode_STY_ZER = 8'h84;
    parameter opcode_BCC = 8'h90;
    parameter opcode_CLV = 8'hB8;
    parameter opcode_SEI = 8'h78;
    parameter opcode_CLI = 8'h58;
    parameter opcode_BVS = 8'h70;
    parameter opcode_TSX = 8'hBA;
    parameter opcode_BVC = 8'h50;
    parameter opcode_BMI = 8'h30;
    parameter opcode_BPL = 8'h10;
    parameter opcode_ADC_ZER = 8'h65;
    parameter opcode_SBC_ZER = 8'hE5;
    parameter opcode_CMP_ZER = 8'hC5;
    

    //Memory definitions
    parameter reset_vector = 16'h0000;

    //State machine definitions 
    
    //fetch decode states
    parameter fetch = 2'b00;                //set fetch to active
    parameter decode = 2'b01;               //set decode to active
    parameter fetch_wait = 2'b10;           //sets a 1 cycle delay then moves to fetch mode
    parameter decode_wait = 2'b11;          //sets a 1 cycle delay then move to decode mode

    //opcode state machine
    
    parameter decode_phase1 = 4'b0000;           //first phase of reading for lda
    parameter decode_phase2 = 4'b0001;           //second phase of reading for lda
    parameter decode_phase3 = 4'b0010;           //third phase of reading for lda
    parameter decode_phase4 = 4'b0011;           //fourth phase of reading for lda
    parameter decode_phase5 = 4'b0101;            //set 1 cycle delay after lda and move back to fetch after
    parameter decode_phase6 = 4'b0111;
    parameter jmp_wait = 4'b0100;             //set 1 cycle delay after jmp and move back to fetch after

    parameter boot_ram_clear = 2'b00;
    parameter boot_rom_copy = 2'b01;
    parameter boot_complete = 2'b10;

    always @(posedge slower_clk or posedge reset) begin
        
        if (reset) begin
            pc <= reset_vector;             //on reset move to reset memory location
            led_reg <= 4'b0101;
            fetch_decode <= fetch_wait;
            decode_phase <= decode_phase1;
            x <= 8'h00;
            y <= 8'h00;
            a <= 8'h00;
            sp <= 8'hFF;
            no_advance <= 1'b1;
        end else begin    
            data_in = ram[pc[10:0]];
            case(fetch_decode)
                fetch: begin
                    led_reg <= a[5:0];
                    set_write_enable <= 1'b0;
                    ir <= data_in;          //read data from memory module
                    decode_phase <= decode_phase1;
                    fetch_decode <= decode_wait;
                    no_advance <= 1'b0;
                end
                decode: begin
                    case(ir)
                        opcode_NOP: begin
                            no_advance <= 1'b1;
                            fetch_decode <= fetch_wait;
                        end
                        opcode_LDA_IMM: begin
                            a <= data_in;
                            zero_flag <= (data_in == 8'h00);
                            negative_flag <= data_in[7];
                            fetch_decode <= fetch_wait;
                            decode_phase <= decode_phase1;
                        end
                        opcode_LDA_ZER: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    return <= pc;
                                    addr_low <= data_in;
                                    pc <= {8'h00, addr_low};
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode;
                                end
                                decode_phase2: begin
                                    a <= data_in;
                                    if (data_in == 8'h00)
                                        zero_flag <= 1;
                                    else
                                        zero_flag <= 0;
                                    negative_flag <= data_in[7];
                                    pc <= return;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_LDX_ZER: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    return <= pc;
                                    addr_low <= data_in;
                                    pc <= {8'h00, addr_low};
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode;
                                end
                                decode_phase2: begin
                                    x <= data_in;
                                    if (data_in == 8'h00)
                                        zero_flag <= 1;
                                    else
                                        zero_flag <= 0;
                                    negative_flag <= data_in[7];
                                    pc <= return;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_LDY_ZER: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    return <= pc;
                                    addr_low <= data_in;
                                    pc <= {8'h00, addr_low};
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode;
                                end
                                decode_phase2: begin
                                    y <= data_in;
                                    if (data_in == 8'h00)
                                        zero_flag <= 1;
                                    else
                                        zero_flag <= 0;
                                    negative_flag <= data_in[7];
                                    pc <= return;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_JMP: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    addr_low <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase2: begin
                                    addr_high <= data_in;
                                    pc <= {data_in, addr_low};
                                     //pc <= 16'h0000; 
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_SEC: begin
                            carry_flag <= 1'b1;
                            fetch_decode <= fetch_wait;
                            no_advance <= 1'b1;
                        end
                        opcode_SEI: begin
                            interrupt_disable_flag <= 1'b1;
                            fetch_decode <= fetch_wait;
                            no_advance <= 1'b1;
                        end

                        opcode_CLI: begin
                            interrupt_disable_flag <= 1'b0;
                            fetch_decode <= fetch_wait;
                            no_advance <= 1'b1;
                        end
                        opcode_INX: begin
                            x <= x + 1;
                            no_advance <= 1'b1;
                            if ((x + 1) == 8'h00)
                                zero_flag <= 1;
                            else
                                zero_flag <= 0;
                            fetch_decode <= fetch_wait;
                        end
                        opcode_TSX: begin
                            x <= sp;
                            zero_flag <= (sp == 8'h00);
                            negative_flag <= sp[7];
                            fetch_decode <= fetch_wait;
                            no_advance <= 1'b1;
                        end
                        opcode_INY: begin
                            y <= y + 1;
                            no_advance <= 1'b1;
                            if ((y + 1) == 8'h00)
                                zero_flag <= 1;
                            else
                                zero_flag <= 0;
                            fetch_decode <= fetch_wait;
                        end
                        opcode_LDX_IMM: begin
                            x <= data_in;
                            if (data_in == 8'h00)
                                zero_flag <= 1;
                            else
                                zero_flag <= 0;
                            fetch_decode <= fetch_wait;
                        end
                         opcode_LDY_IMM: begin
                            y <= data_in;
                            if (data_in == 8'h00)
                                zero_flag <= 1;
                            else
                                zero_flag <= 0;
                            fetch_decode <= fetch_wait;
                        end
                        opcode_TAX: begin
                            x <= a;
                            no_advance <= 1'b1;
                            if (a == 8'h00)
                                zero_flag <= 1;
                            else
                                zero_flag <= 0;
                            fetch_decode <= fetch_wait;
                        end
                        opcode_TAY: begin
                            y <= a;
                            no_advance <= 1'b1;
                            if (a == 8'h00)
                                zero_flag <= 1;
                            else
                                zero_flag <= 0;
                            fetch_decode <= fetch_wait;
                        end
                        opcode_TXA: begin
                            a <= x;
                            no_advance <= 1'b1;
                            if (x == 8'h00)
                                zero_flag <= 1;
                            else
                                zero_flag <= 0;
                            fetch_decode <= fetch_wait;
                        end
                        opcode_TYA: begin
                            a <= y;
                            no_advance <= 1'b1;
                            if (y == 8'h00)
                                zero_flag <= 1;
                            else
                                zero_flag <= 0;
                            fetch_decode <= fetch_wait;
                        end
                        opcode_DEX: begin
                            x <= x - 1;
                            no_advance <= 1'b1;
                            if ((x - 1) == 8'h00)
                                zero_flag <= 1;
                            else
                                zero_flag <= 0;
                            fetch_decode <= fetch_wait;
                        end
                        opcode_DEY: begin
                            y <= y - 1;
                            no_advance <= 1'b1;
                            if ((y - 1) == 8'h00)
                                zero_flag <= 1;
                            else
                                zero_flag <= 0;
                            fetch_decode <= fetch_wait;
                        end
                        opcode_LDA_ABS: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    addr_low <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase2: begin
                                    addr_high <= data_in;
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase3;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase3: begin
                                    return <= pc;
                                    pc <= {addr_high, addr_low};
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase4;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase4: begin
                                    a <= data_in;
                                    pc <= return;
                                    if (data_in == 8'h00)
                                        zero_flag <= 1;
                                    else
                                        zero_flag <= 0;
                                    negative_flag <= data_in[7];
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_LDX_ABS: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    addr_low <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase2: begin
                                    addr_high <= data_in;
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase3;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase3: begin
                                    return <= pc;
                                    pc <= {addr_high, addr_low};
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase4;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase4: begin
                                    x <= data_in;
                                    pc <= return;
                                    if (data_in == 8'h00)
                                        zero_flag <= 1;
                                    else
                                        zero_flag <= 0;
                                    negative_flag <= data_in[7];
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_LDY_ABS: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    addr_low <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase2: begin
                                    addr_high <= data_in;
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase3;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase3: begin
                                    return <= pc;
                                    pc <= {addr_high, addr_low};
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase4;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase4: begin
                                    y <= data_in;
                                    pc <= return;
                                    if (data_in == 8'h00)
                                        zero_flag <= 1;
                                    else
                                        zero_flag <= 0;
                                    negative_flag <= data_in[7];
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_JSR: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    addr_low <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase2: begin
                                    addr_high <= data_in;
                                    decode_phase <= decode_phase3;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase3: begin
                                    set_write_enable <= 1;
                                    return <= pc + 1;
                                    pc <= {8'h01, sp};
                                    no_advance <= 1'b1;
                                    data_out <= return[15:8];
                                    decode_phase <= decode_phase4;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase4: begin
                                    set_write_enable <= 1;
                                    sp <= sp - 1;
                                    pc <= {8'h01, sp};
                                    no_advance <= 1'b1;
                                    data_out <= return[7:0];
                                    decode_phase <= decode_phase5;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase5: begin
                                    set_write_enable <= 0;
                                    sp <= sp - 1;
                                    pc <= {addr_high, addr_low};
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_STA_ABS: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    addr_low <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase2: begin
                                    addr_high <= data_in;
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase3;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase3: begin
                                    return <= pc;
                                    pc <= {addr_high, addr_low};
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase4;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase4: begin
                                    set_write_enable <= 1'b1;
                                    data_out <= a;
                                    decode_phase <= decode_phase5;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase5: begin
                                    set_write_enable <= 1'b0;
                                    pc <= return;
                                    decode_phase <= decode_phase1;
                                    decode_phase <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_PHA: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    return <= pc;
                                    pc <= {8'h01, sp};
                                    no_advance <= 1'b1;
                                    set_write_enable <= 1'b1;
                                    fetch_decode <= decode_wait;
                                    decode_phase <= decode_phase2;
                                end
                                decode_phase2: begin
                                    data_out <= a;
                                    fetch_decode <= decode_wait;
                                    decode_phase <= decode_phase3;
                                end
                                decode_phase3: begin
                                    pc <= return;
                                    sp <= sp - 1;
                                    set_write_enable <= 1'b0;
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_PLA: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    return <= pc;
                                    pc <= {8'h01, sp};
                                    no_advance <= 1'b1;
                                    fetch_decode <= decode_wait;
                                    decode_phase <= decode_phase2;
                                end
                                decode_phase2: begin
                                    sp <= sp + 1;
                                    a <= data_in;
                                    if (data_in == 8'h00)
                                        zero_flag <= 1;
                                    else
                                        zero_flag <= 0;
                                    fetch_decode <= decode_wait;
                                    decode_phase <= decode_phase3;
                                end
                                decode_phase3: begin
                                    pc <= return;
                                    no_advance <= 1'b1;
                                    fetch_decode <= fetch_wait;
                                    decode_phase <= decode_phase1;
                                end
                            endcase
                        end
                        opcode_RTS: begin
                            case(fetch_decode)
                                decode_phase1: begin
                                    pc <= {8'h01, sp};
                                    no_advance <= 1'b1;
                                    fetch_decode <= decode_wait;
                                    decode_phase <= decode_phase2;
                                end
                                decode_phase2: begin
                                    addr_low <= data_in;
                                    sp <= sp + 1;
                                    pc <= {8'h01, sp + 1};
                                    no_advance <= 1'b1;
                                    fetch_decode <= decode_wait;
                                    decode_phase <= decode_phase3;
                                end
                                decode_phase3: begin
                                    addr_high <= data_in;
                                    sp <= sp + 1;
                                    fetch_decode <= decode_wait;
                                    decode_phase <= decode_phase4;
                                end
                                decode_phase4: begin
                                    pc <= {addr_high, addr_low} + 1;
                                    no_advance <= 1'b1;
                                    fetch_decode <= fetch_wait;
                                    decode_phase <= decode_phase1;
                                end
                            endcase
                        end
                        opcode_ADC_IMM: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    temp_math <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end 
                                decode_phase2: begin
                                    {carry_flag, a} <= a + temp_math + carry_flag;
                                    overflow_flag <= (~(a ^ temp_math) & (a ^ (a + temp_math + carry_flag)))[7];
                                    if ((a + temp_math + carry_flag) == 8'h00)
                                        zero_flag <= 1;
                                    else
                                        zero_flag <= 0;
                                    negative_flag <= result[7];
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_ADC_ZER: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    return <= pc;
                                    pc <= {8'h00, data_in};
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end 
                                decode_phase2: begin
                                    temp_math <= data_in;
                                    decode_phase <= decode_phase3;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase3: begin
                                    {carry_flag, a} <= a + temp_math + carry_flag;
                                    overflow_flag <= (~(a ^ temp_math) & (a ^ (a + temp_math + carry_flag)))[7];
                                    if ((a + temp_math + carry_flag) == 8'h00)
                                        zero_flag <= 1;
                                    else
                                        zero_flag <= 0;
                                    negative_flag <= (a + (temp_math) + carry_flag)[7];
                                    pc <= return;
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_SBC_ZER: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    return <= pc;
                                    pc <= {8'h00, data_in};
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end 
                                decode_phase2: begin
                                    temp_math <= data_in;
                                    decode_phase <= decode_phase3;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase3: begin
                                    {carry_flag, a} <= a + (~temp_math) + carry_flag;
                                    overflow_flag <= ((a ^ temp_math) & (a ^ (a + (~temp_math) + carry_flag)))[7];
                                    if ((a + (~temp_math) + carry_flag) == 8'h00)
                                        zero_flag <= 1;
                                    else
                                        zero_flag <= 0;
                                    negative_flag <= (a + (~temp_math) + carry_flag)[7];
                                    pc <= return;
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_CMP_ZER: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    return <= pc;
                                    pc <= {8'h00, data_in};
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end 
                                decode_phase2: begin
                                    temp_math <= data_in;
                                    decode_phase <= decode_phase3;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase3: begin
                                    carry_flag <= (a >= temp_math);
                                    if ((a + (~temp_math) + 1) == 8'h00)
                                        zero_flag <= 1;
                                    else
                                        zero_flag <= 0;
                                    negative_flag <= (a + (~temp_math) + 1)[7];
                                    pc <= return;
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_CLC: begin
                            carry_flag <= 1'b0;
                            fetch_decode <= fetch_wait;
                            no_advance <= 1'b1;
                        end
                        opcode_BEQ: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    branch_offset <= data_in;
                                    fetch_decode <= decode_wait;
                                    no_advance <=1'b1;
                                    decode_phase <=  decode_phase2;        
                                end
                                decode_phase2: begin
                                    if (zero_flag == 1) begin
                                         pc <= pc + $signed({{8{branch_offset[7]}}, branch_offset});
                                         no_advance <=1'b1;
                                    end else begin
                                        no_advance <=1'b0;
                                    end
                                    fetch_decode <= fetch_wait;
                                    decode_phase <= decode_phase1;
                                end
                            endcase
                        end
                        opcode_BVC: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    branch_offset <= data_in;
                                    fetch_decode <= decode_wait;
                                    no_advance <=1'b1;
                                    decode_phase <=  decode_phase2;        
                                end
                                decode_phase2: begin
                                    if (overflow_flag == 0) begin
                                         pc <= pc + $signed({{8{branch_offset[7]}}, branch_offset});
                                         no_advance <=1'b1;
                                    end else begin
                                        no_advance <=1'b0;
                                    end
                                    fetch_decode <= fetch_wait;
                                    decode_phase <= decode_phase1;
                                end
                            endcase
                        end
                        opcode_BMI: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    branch_offset <= data_in;
                                    fetch_decode <= decode_wait;
                                    no_advance <=1'b1;
                                    decode_phase <=  decode_phase2;        
                                end
                                decode_phase2: begin
                                    if (negative_flag == 1) begin
                                         pc <= pc + $signed({{8{branch_offset[7]}}, branch_offset});
                                         no_advance <=1'b1;
                                    end else begin
                                        no_advance <=1'b0;
                                    end
                                    fetch_decode <= fetch_wait;
                                    decode_phase <= decode_phase1;
                                end
                            endcase
                        end
                        opcode_BPL: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    branch_offset <= data_in;
                                    fetch_decode <= decode_wait;
                                    no_advance <=1'b1;
                                    decode_phase <=  decode_phase2;        
                                end
                                decode_phase2: begin
                                    if (negative_flag == 0) begin
                                         pc <= pc + $signed({{8{branch_offset[7]}}, branch_offset});
                                         no_advance <=1'b1;
                                    end else begin
                                        no_advance <=1'b0;
                                    end
                                    fetch_decode <= fetch_wait;
                                    decode_phase <= decode_phase1;
                                end
                            endcase
                        end
                        opcode_BCC: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    branch_offset <= data_in;
                                    fetch_decode <= decode_wait;
                                    no_advance <=1'b1;
                                    decode_phase <=  decode_phase2;        
                                end
                                decode_phase2: begin
                                    if (carry_flag == 0) begin
                                         pc <= pc + $signed({{8{branch_offset[7]}}, branch_offset});
                                         no_advance <=1'b1;
                                    end else begin
                                        no_advance <=1'b0;
                                    end
                                    fetch_decode <= fetch_wait;
                                    decode_phase <= decode_phase1;
                                end
                            endcase
                        end
                      opcode_BNE: begin
                        case(decode_phase)
                            decode_phase1: begin
                                branch_offset <= data_in;
                                fetch_decode <= decode_wait;
                                decode_phase <= decode_phase2;
                                no_advance <= 1'b0;  // Allow normal PC increment for operand fetch
                            end
                            decode_phase2: begin
                                if (zero_flag == 0) begin
                                    pc <= pc + $signed({{8{branch_offset[7]}}, branch_offset});
                                    no_advance <= 1'b1;  // manual PC change: prevent next auto increment
                                end else begin  
                                    no_advance <= 1'b0;  // let PC increment normally
                                end
                                fetch_decode <= fetch_wait;
                                decode_phase <= decode_phase1;
                            end
                        endcase
                    end
                    opcode_BVS: begin
                        case(decode_phase)
                            decode_phase1: begin
                                branch_offset <= data_in;
                                fetch_decode <= decode_wait;
                                decode_phase <= decode_phase2;
                                no_advance <= 1'b0;  // Allow normal PC increment for operand fetch
                            end
                            decode_phase2: begin
                                if (overflow_flag == 1) begin
                                    pc <= pc + $signed({{8{branch_offset[7]}}, branch_offset});
                                    no_advance <= 1'b1;  // manual PC change: prevent next auto increment
                                end else begin  
                                    no_advance <= 1'b0;  // let PC increment normally
                                end
                                fetch_decode <= fetch_wait;
                                decode_phase <= decode_phase1;
                            end
                        endcase
                    end
                    opcode_CMP_IMM: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    temp_math <= data_in;
                                    fetch_decode <= decode_wait;
                                    decode_phase <= decode_phase2;
                                    no_advance <= 1'b1;
                                end
                                decode_phase2: begin
                                    if ((a - temp_math) == 8'h00)
                                        zero_flag <= 1'b1;
                                    else
                                        zero_flag <= 1'b0;

                                    if (a >= temp_math)
                                        carry_flag <= 1'b1;
                                    else
                                        carry_flag <= 1'b0;

                                    if ((a - temp_math) & 8'h80) // manually checking MSB (bit 7) directly
                                        negative_flag <= 1'b1;
                                    else
                                        negative_flag <= 1'b0;

                                    fetch_decode <= fetch_wait;
                                    decode_phase <= decode_phase1;
                                    no_advance <= 1'b0;
                                end
                            endcase
                        end
                        opcode_SBC_IMM: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    temp_math <= data_in;
                                    fetch_decode <= decode_wait;
                                    decode_phase <= decode_phase2;
                                end
                                decode_phase2: begin
                                    temp_result <= a + (~temp_math) + carry_flag;
                                    a <= temp_result;
                                    zero_flag <= (temp_result == 8'h00);
                                    carry_flag <= (a >= (temp_math + (~carry_flag))); // carry = no borrow
                                    overflow_flag <= ((a ^ temp_math) & (a ^ temp_result))[7];
                                    negative_flag <= temp_result[7];
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                    no_advance <= 1'b01;
                                end
                            endcase
                        end
                        opcode_CLV: begin
                            overflow_flag <= 1'b0;
                            fetch_decode <= fetch_wait;
                            no_advance <= 1'b1;
                        end
                        opcode_AND_IMM: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    temp_math <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase2: begin
                                  and_result <= a & temp_math;
                                  a <= and_result;
                                  zero_flag <= (and_result == 8'h00);
                                  negative_flag <= and_result[7];
                                  fetch_decode <= fetch_wait;
                                  decode_phase <= decode_phase1;
                                  no_advance <= 1'b1;
                                end
                            endcase
                        end
                        opcode_ORA_IMM: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    temp_math <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase2: begin
                                    or_result <= a | temp_math;
                                    a <= or_result;
                                    zero_flag <= (or_result == 8'h00);
                                    negative_flag <= or_result[7];
                                    fetch_decode <= fetch_wait;
                                    decode_phase <= decode_phase1;
                                    no_advance <= 1'b1;
                                end
                            endcase
                        end
                        opcode_EOR_IMM: begin
                            case(decode_phase)
                                decode_phase1: begin
                                    temp_math <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end
                               decode_phase2: begin
                                   or_result <= a ^ temp_math;
                                   a <= or_result;
                                   zero_flag <= (or_result == 8'h00);
                                   negative_flag <= or_result[7];
                                   fetch_decode <= fetch_wait;
                                   decode_phase <= decode_phase1;
                                   no_advance <= 1'b1;
                                end
                                default: pc <= 16'h0000;
                            endcase
                        end
                        opcode_ASL: begin
                            a <= {a[6:0], 1'b0};         // left-shift, LSB becomes 0
                            carry_flag <= a[7];          // old MSB into carry
                            zero_flag <= ({a[6:0], 1'b0} == 8'h00);
                            negative_flag <= a[7];
                            fetch_decode <= fetch_wait;
                            no_advance <= 1'b1;
                        end
                        opcode_LDA_INX:begin
                            case(decode_phase)
                                decode_phase1: begin
                                    addr_low <= data_in;
                                    return <= pc;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase2: begin
                                    pc <= {8'h00, (x + addr_low) & 8'hFF};
                                    no_advance = 1'b1;
                                    decode_phase <= decode_phase3;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase3: begin
                                    addr_low <= data_in;
                                    decode_phase <= decode_phase4;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase4: begin
                                    addr_high <= data_in;
                                    decode_phase <= decode_phase5;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase5: begin
                                    pc <= {addr_high, addr_low};
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase6;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase6: begin
                                    a <= data_in;
                                    zero_flag <= (data_in == 8'h00);
                                    negative_flag <= data_in[7];  // MSB determines negative
                                    pc <= return;
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_STA_INX:begin
                            case(decode_phase)
                                decode_phase1: begin
                                    addr_low <= data_in;
                                    return <= pc;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase2: begin
                                    pc <= {8'h00, (x + addr_low) & 8'hFF};
                                    no_advance = 1'b1;
                                    decode_phase <= decode_phase3;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase3: begin
                                    addr_low <= data_in;
                                    decode_phase <= decode_phase4;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase4: begin
                                    addr_high <= data_in;
                                    decode_phase <= decode_phase5;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase5: begin
                                    pc <= {addr_high, addr_low};
                                    no_advance <= 1'b1;
                                    set_write_enable <= 1'b1;
                                    decode_phase <= decode_phase6;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase6: begin
                                    data_out <= a;
                                    pc <= return;
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_LDA_INY:begin
                            case(decode_phase)
                                decode_phase1: begin
                                    addr_low <= data_in;
                                    return <= pc;
                                    
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase2: begin
                                    pc <= {8'h00, addr_low};
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase3;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase3: begin
                                    addr_low <= data_in;
                                    decode_phase <= decode_phase4;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase4: begin
                                    addr_high <= data_in;
                                    decode_phase <= decode_phase5;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase5: begin
                                    pc <= {addr_high, addr_low} + y;
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase6;
                                    fetch_decode <= decode_wait;
                                end
                                decode_phase6: begin
                                    a <= data_in;
                                    pc <= return;
                                    zero_flag <= (data_in == 8'h00);
                                    negative_flag <= data_in[7];  // MSB determines negative
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_STA_ZER: begin
                            case (decode_phase)
                                decode_phase1: begin
                                    addr_low <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;        
                                end
                                decode_phase2: begin
                                    pc <= {8'h00, addr_low};
                                    no_advance <= 1'b1;
                                    fetch_decode <= decode_wait;
                                    decode_phase <= decode_phase3;
                                    write_enable <= 1'b1;
                                end    
                                decode_phase3: begin
                                    data_out <= a;
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_STX_ZER: begin
                            case (decode_phase)
                                decode_phase1: begin
                                    addr_low <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;        
                                end
                                decode_phase2: begin
                                    pc <= {8'h00, addr_low};
                                    no_advance <= 1'b1;
                                    fetch_decode <= decode_wait;
                                    decode_phase <= decode_phase3;
                                    write_enable <= 1'b1;
                                end    
                                decode_phase3: begin
                                    data_out <= x;
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                        opcode_STY_ZER: begin
                            case (decode_phase)
                                decode_phase1: begin
                                    addr_low <= data_in;
                                    decode_phase <= decode_phase2;
                                    fetch_decode <= decode_wait;        
                                end
                                decode_phase2: begin
                                    pc <= {8'h00, addr_low};
                                    no_advance <= 1'b1;
                                    fetch_decode <= decode_wait;
                                    decode_phase <= decode_phase3;
                                    write_enable <= 1'b1;
                                end    
                                decode_phase3: begin
                                    data_out <= y;
                                    no_advance <= 1'b1;
                                    decode_phase <= decode_phase1;
                                    fetch_decode <= fetch_wait;
                                end
                            endcase
                        end
                    endcase
                end
                fetch_wait: begin
                    if (no_advance != 1'b1) begin
                        pc <= pc + 1;
                    end
                    no_advance <= 1'b0;
                    fetch_decode <= fetch;
                end
                decode_wait: begin
                    if (no_advance != 1'b1) begin
                        pc <= pc + 1;
                    end
                    fetch_decode <= decode;
                    no_advance <= 1'b0;
                end
                default: fetch_decode <= fetch_wait;
            endcase
        end
    end
//assign data_in = ram[pc];
assign led = ~led_reg[5:0];
  
endmodule