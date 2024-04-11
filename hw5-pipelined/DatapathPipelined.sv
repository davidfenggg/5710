`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// insns are 32 bits in RV32IM
`define INSN_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`ifndef RISCV_FORMAL
`include "../hw2b/cla.sv"
`include "../hw3-singlecycle/RvDisassembler.sv"
`include "../hw4-multicycle/divider_unsigned_pipelined.sv"
`endif

module Disasm #(
    byte PREFIX = "D"
) (
    input wire [31:0] insn,
    output wire [(8*32)-1:0] disasm
);
  // synthesis translate_off
  // this code is only for simulation, not synthesis
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic. Also,
  // string needs to be reversed to render correctly.
  genvar i;
  for (i = 3; i < 32; i = i + 1) begin : gen_disasm
    assign disasm[((i+1-3)*8)-1-:8] = disasm_string[31-i];
  end
  assign disasm[255-:8] = PREFIX;
  assign disasm[247-:8] = ":";
  assign disasm[239-:8] = " ";
  // synthesis translate_on
endmodule

module RegFile (
    input logic [4:0] rd,
    input logic [`REG_SIZE] rd_data,
    input logic [4:0] rs1,
    output logic [`REG_SIZE] rs1_data,
    input logic [4:0] rs2,
    output logic [`REG_SIZE] rs2_data,

    input logic clk,
    input logic we,
    input logic rst
);
  localparam int NumRegs = 32;
  logic [`REG_SIZE] regs[NumRegs];

  assign regs[0] = 32'd0;

  assign rs1_data = regs[rs1];
  assign rs2_data = regs[rs2];

  genvar i;
  for (i = 1; i < 32; i = i + 1) begin 
    always_ff @( posedge clk ) begin
      if (rst) begin
        regs[i] <= 32'd0;
      end else begin
        if (we && rd == i) begin
          regs[i] <= rd_data;
        end
      end
    end
  end

endmodule

/**
 * This enum is used to classify each cycle as it comes through the Writeback stage, identifying
 * if a valid insn is present or, if it is a stall cycle instead, the reason for the stall. The
 * enum values are mutually exclusive: only one should be set for any given cycle. These values
 * are compared against the trace-*.json files to ensure that the datapath is running with the
 * correct timing.
 *
 * You will need to set these values at various places within your pipeline, and propagate them
 * through the stages until they reach Writeback where they can be checked.
 */
typedef enum {
  /** invalid value, this should never appear after the initial reset sequence completes */
  CYCLE_INVALID = 0,
  /** a stall cycle that arose from the initial reset signal */
  CYCLE_RESET = 1,
  /** not a stall cycle, a valid insn is in Writeback */
  CYCLE_NO_STALL = 2,
  /** a stall cycle that arose from a taken branch/jump */
  CYCLE_TAKEN_BRANCH = 4,

  // the values below are only needed in HW5B

  /** a stall cycle that arose from a load-to-use stall */
  CYCLE_LOAD2USE = 8,
  /** a stall cycle that arose from a div/rem-to-use stall */
  CYCLE_DIV2USE = 16,
  /** a stall cycle that arose from a fence.i insn */
  CYCLE_FENCEI = 32
} cycle_status_e;

/** state at the start of Decode stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
} stage_decode_t;

/** state at the start of Execute stage */
typedef struct packed {
  logic [`REG_SIZE] pc_x;
  logic [`INSN_SIZE] insn;
  logic [`REG_SIZE] rs1_data, rs2_data;
  logic [46:0] insn_one_hot;
  cycle_status_e cycle_status;
} stage_execute_t;

/** state at the start of Memory stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`REG_SIZE] rd_data; // for store instructions
  logic [`INSN_SIZE] insn;
  logic reg_we_m; //we for mem
  logic [4:0] rd;
  logic halt_m;
  logic [`REG_SIZE] addr_to_dmem_m;
  logic [`REG_SIZE] rs1_data_m;
  logic [`REG_SIZE] rs2_data_m;
  logic [`REG_SIZE] imm_i_sext_m;
  logic [`REG_SIZE] imm_s_sext_m;
  logic [46:0] insn_one_hot;
  logic [`REG_SIZE] rs1_add_imm_m;
  logic [`REG_SIZE] rs1_add_imm_s;
  cycle_status_e cycle_status;
} stage_memory_t;

/** state at the start of Write-back stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`REG_SIZE] rd_data; //for write data
  logic [`INSN_SIZE] insn;
  logic reg_we_w;
  logic [4:0] rd;
  logic halt_w;
  cycle_status_e cycle_status;
} stage_writeback_t;

module DatapathPipelined (
    input wire clk,
    input wire rst,
    output logic [`REG_SIZE] pc_to_imem,
    input wire [`INSN_SIZE] insn_from_imem,
    // dmem is read/write
    output logic [`REG_SIZE] addr_to_dmem,
    input wire [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0] store_we_to_dmem,

    output logic halt,

    // The PC of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`REG_SIZE] trace_writeback_pc,
    // The bits of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`INSN_SIZE] trace_writeback_insn,
    // The status of the insn (or stall) currently in Writeback. See cycle_status_e enum for valid values.
    output cycle_status_e trace_writeback_cycle_status
);

  // opcodes - see section 19 of RiscV spec
  localparam bit [`OPCODE_SIZE] OpcodeLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeJalr = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpcodeMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpcodeJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpcodeRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpcodeAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpcodeLui = 7'b01_101_11;

  // cycle counter, not really part of any stage but useful for orienting within GtkWave
  // do not rename this as the testbench uses this value
  logic [`REG_SIZE] cycles_current;
  always_ff @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end

  /*****************************************************/
  /*                    FETCH STAGE                    */
  /*****************************************************/

  logic [`REG_SIZE] f_pc_current;
  wire [`REG_SIZE] f_insn;
  cycle_status_e f_cycle_status;

  // program counter
  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current <= 32'd0;
      // NB: use CYCLE_NO_STALL since this is the value that will persist after the last reset cycle
      f_cycle_status <= CYCLE_NO_STALL;
    end else if (load_use_stall || div_stall_required) begin
    end else if (store_in_execute || store_in_memory) begin
    end else if (branch_taken) begin
      f_pc_current <= pc_x;
      f_cycle_status <= CYCLE_NO_STALL;
    end else begin
      f_cycle_status <= CYCLE_NO_STALL;
      f_pc_current <= f_pc_current + 4;
    end
  end
  // send PC to imem
  assign pc_to_imem = f_pc_current;
  assign f_insn = insn_from_imem;

  // Here's how to disassemble an insn into a string you can view in GtkWave.
  // Use PREFIX to provide a 1-character tag to identify which stage the insn comes from.
  wire [255:0] f_disasm;
  Disasm #(
      .PREFIX("F")
  ) disasm_0fetch (
      .insn  (f_insn),
      .disasm(f_disasm)
  );

  /*****************************************************/
  /*                    DECODE STAGE                   */
  /*****************************************************/
  // this shows how to package up state in a `struct packed`, and how to pass it between stages
  stage_decode_t decode_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      decode_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET
      };
    end else if (load_use_stall || div_stall_required) begin
      decode_state <= '{
        pc: decode_state.pc,
        insn: decode_state.insn,
        cycle_status: decode_state.cycle_status
      };
    end else if (branch_taken) begin
      decode_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_TAKEN_BRANCH
      };
    end else if (store_in_execute || store_in_memory) begin
      decode_state <= '{
        pc: decode_state.pc,
        insn: decode_state.insn,
        cycle_status: decode_state.cycle_status
      };
    end else begin
      decode_state <= '{
        pc: f_pc_current,
        insn: f_insn,
        cycle_status: f_cycle_status
      };
    end
  end

  wire store_in_execute = (insn_opcode_x == OpcodeStore && insn_opcode == OpcodeMiscMem);
  wire store_in_memory = (insn_opcode_m == OpcodeStore && insn_opcode == OpcodeMiscMem);

  wire [255:0] d_disasm;
  Disasm #(
      .PREFIX("D")
  ) disasm_1decode (
      .insn  (decode_state.insn),
      .disasm(d_disasm)
  );

  // components of the instruction
  wire [6:0] insn_funct7_d;
  wire [4:0] insn_rs2_d;
  wire [4:0] insn_rs1_d;
  wire [2:0] insn_funct3_d;
  wire [4:0] insn_rd_d;
  wire [`OPCODE_SIZE] insn_opcode;

  // split R-type instruction - see section 2.2 of RiscV spec
  assign {insn_funct7_d, insn_rs2_d, insn_rs1_d, insn_funct3_d, insn_rd_d, insn_opcode} = decode_state.insn;

  // 0 
  wire insn_lui = insn_opcode == OpcodeLui;
  wire insn_auipc = insn_opcode == OpcodeAuipc;
  wire insn_jal = insn_opcode == OpcodeJal;
  wire insn_jalr = insn_opcode == OpcodeJalr;
  // 3

  // 4
  wire insn_beq = insn_opcode == OpcodeBranch && decode_state.insn[14:12] == 3'b000;
  wire insn_bne = insn_opcode == OpcodeBranch && decode_state.insn[14:12] == 3'b001;
  wire insn_blt = insn_opcode == OpcodeBranch && decode_state.insn[14:12] == 3'b100;
  wire insn_bge = insn_opcode == OpcodeBranch && decode_state.insn[14:12] == 3'b101;
  wire insn_bltu = insn_opcode == OpcodeBranch && decode_state.insn[14:12] == 3'b110;
  wire insn_bgeu = insn_opcode == OpcodeBranch && decode_state.insn[14:12] == 3'b111;
  // 9

  // 10
  wire insn_lb = insn_opcode == OpcodeLoad && decode_state.insn[14:12] == 3'b000;
  wire insn_lh = insn_opcode == OpcodeLoad && decode_state.insn[14:12] == 3'b001;
  wire insn_lw = insn_opcode == OpcodeLoad && decode_state.insn[14:12] == 3'b010;
  wire insn_lbu = insn_opcode == OpcodeLoad && decode_state.insn[14:12] == 3'b100;
  wire insn_lhu = insn_opcode == OpcodeLoad && decode_state.insn[14:12] == 3'b101;
  // 14

  // 15
  wire insn_sb = insn_opcode == OpcodeStore && decode_state.insn[14:12] == 3'b000;
  wire insn_sh = insn_opcode == OpcodeStore && decode_state.insn[14:12] == 3'b001;
  wire insn_sw = insn_opcode == OpcodeStore && decode_state.insn[14:12] == 3'b010;
  // 17

  // 18
  wire insn_addi = insn_opcode == OpcodeRegImm && decode_state.insn[14:12] == 3'b000;
  wire insn_slti = insn_opcode == OpcodeRegImm && decode_state.insn[14:12] == 3'b010;
  wire insn_sltiu = insn_opcode == OpcodeRegImm && decode_state.insn[14:12] == 3'b011;
  wire insn_xori = insn_opcode == OpcodeRegImm && decode_state.insn[14:12] == 3'b100;
  wire insn_ori = insn_opcode == OpcodeRegImm && decode_state.insn[14:12] == 3'b110;
  wire insn_andi = insn_opcode == OpcodeRegImm && decode_state.insn[14:12] == 3'b111;
  // 23

  // 24
  wire insn_slli = insn_opcode == OpcodeRegImm && decode_state.insn[14:12] == 3'b001 && decode_state.insn[31:25] == 7'd0;
  wire insn_srli = insn_opcode == OpcodeRegImm && decode_state.insn[14:12] == 3'b101 && decode_state.insn[31:25] == 7'd0;
  wire insn_srai = insn_opcode == OpcodeRegImm && decode_state.insn[14:12] == 3'b101 && decode_state.insn[31:25] == 7'b0100000;
  // 26

  // 27
  wire insn_add = insn_opcode == OpcodeRegReg && decode_state.insn[14:12] == 3'b000 && decode_state.insn[31:25] == 7'd0;
  wire insn_sub  = insn_opcode == OpcodeRegReg && decode_state.insn[14:12] == 3'b000 && decode_state.insn[31:25] == 7'b0100000;
  wire insn_sll = insn_opcode == OpcodeRegReg && decode_state.insn[14:12] == 3'b001 && decode_state.insn[31:25] == 7'd0;
  wire insn_slt = insn_opcode == OpcodeRegReg && decode_state.insn[14:12] == 3'b010 && decode_state.insn[31:25] == 7'd0;
  wire insn_sltu = insn_opcode == OpcodeRegReg && decode_state.insn[14:12] == 3'b011 && decode_state.insn[31:25] == 7'd0;
  wire insn_xor = insn_opcode == OpcodeRegReg && decode_state.insn[14:12] == 3'b100 && decode_state.insn[31:25] == 7'd0;
  wire insn_srl = insn_opcode == OpcodeRegReg && decode_state.insn[14:12] == 3'b101 && decode_state.insn[31:25] == 7'd0;
  wire insn_sra  = insn_opcode == OpcodeRegReg && decode_state.insn[14:12] == 3'b101 && decode_state.insn[31:25] == 7'b0100000;
  wire insn_or = insn_opcode == OpcodeRegReg && decode_state.insn[14:12] == 3'b110 && decode_state.insn[31:25] == 7'd0;
  wire insn_and = insn_opcode == OpcodeRegReg && decode_state.insn[14:12] == 3'b111 && decode_state.insn[31:25] == 7'd0;
  // 36

  // 37
  wire insn_mul    = insn_opcode == OpcodeRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b000;
  wire insn_mulh   = insn_opcode == OpcodeRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b001;
  wire insn_mulhsu = insn_opcode == OpcodeRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b010;
  wire insn_mulhu  = insn_opcode == OpcodeRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b011;
  wire insn_div    = insn_opcode == OpcodeRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b100;
  wire insn_divu   = insn_opcode == OpcodeRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b101;
  wire insn_rem    = insn_opcode == OpcodeRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b110;
  wire insn_remu   = insn_opcode == OpcodeRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b111;
  // 44

  // 45
  wire insn_ecall = insn_opcode == OpcodeEnviron && decode_state.insn[31:7] == 25'd0;
  wire insn_fence = insn_opcode == OpcodeMiscMem;
  // 46

  // TODO: need to propagate what insturction was sent (maybe through one hot?)
  wire [46:0] insn_one_hot = {insn_fence, insn_lui, insn_auipc, insn_jal, insn_jalr, insn_beq, insn_bne, insn_blt, 
  insn_bge, insn_bltu, insn_bgeu, insn_lb, insn_lh, insn_lw, insn_lbu, insn_lhu, insn_sb, insn_sh, 
  insn_sw, insn_addi, insn_slti, insn_sltiu, insn_xori, insn_ori, insn_andi, insn_slli, insn_srli, 
  insn_srai, insn_add, insn_sub, insn_sll, insn_slt, insn_sltu, insn_xor, insn_srl, insn_sra, insn_or, 
  insn_and, insn_mul, insn_mulh, insn_mulhsu, insn_mulhu, insn_div, insn_divu, insn_rem, insn_remu, 
  insn_ecall};
  
  logic illegal_insn;
  logic [`REG_SIZE] rs1_data_d, rs2_data_d;
  logic branch_taken;

  //WD BYPASS:
  logic [`REG_SIZE] rs1_data_decoded, rs2_data_decoded;

  always_comb begin
    rs1_data_decoded = (writeback_state.reg_we_w && writeback_state.rd == insn_rs1_d && writeback_state.rd != 0) ? writeback_state.rd_data : rs1_data_d;
    rs2_data_decoded = (writeback_state.reg_we_w && writeback_state.rd == insn_rs2_d && writeback_state.rd != 0) ? writeback_state.rd_data : rs2_data_d;
  end

  // Load-Use Stalling
  wire load_in_execute = insn_opcode_x == OpcodeLoad;
  wire rs1_conflict = (insn_rs1_d == insn_rd_x & insn_rs1_d != 0);
  wire rs2_conflict = (insn_rs2_d == insn_rd_x & insn_rs2_d != 0);
  wire r_s_b_insn = (insn_opcode == OpcodeRegReg | insn_opcode == OpcodeStore | insn_opcode == OpcodeBranch);
  wire not_store = (insn_opcode != OpcodeStore);
  wire load_use_stall = (load_in_execute & (rs1_conflict | (rs2_conflict & not_store & r_s_b_insn))) ? 1 : 0;

  // TODO: Need to change such that only rs1 and rs2 are modified in this cycle -- rd should be modified during the writeback phase
  RegFile rf (
    .clk(clk),
    .rst(rst),
    .rd(insn_rd_w), // should refer to the insn in the writeback phase
    .rd_data(writeback_state.rd_data), // also refers to the data being produced after execution
    .rs1(insn_rs1_d),
    .rs1_data(rs1_data_d),
    .rs2(insn_rs2_d),
    .rs2_data(rs2_data_d), 
    .we(writeback_state.reg_we_w)
  );

  /*****************************************************/
  /*                   EXECUTE STAGE                   */
  /*****************************************************/

  stage_execute_t execute_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      execute_state <= '{
        pc_x: 0,
        insn: 0,
        rs1_data: 0,
        rs2_data: 0,
        insn_one_hot: 0,
        cycle_status: CYCLE_RESET
      };
    end else if (branch_taken) begin
        execute_state <= '{
          pc_x: 0,
          insn: 0,
          rs1_data: 0, 
          rs2_data: 0, 
          insn_one_hot: 0,
          cycle_status: CYCLE_TAKEN_BRANCH
        };
    end else if (load_use_stall) begin
        execute_state <= '{
          pc_x: 0,
          insn: 0,
          rs1_data: 0, 
          rs2_data: 0, 
          insn_one_hot: 0,
          cycle_status: CYCLE_LOAD2USE
        };
    end else if (store_in_execute || store_in_memory) begin
        execute_state <= '{
          pc_x: 0,
          insn: 0,
          rs1_data: 0, 
          rs2_data: 0, 
          insn_one_hot: 0,
          cycle_status: CYCLE_FENCEI
        };
    end else if (div_stall_required) begin
        execute_state <= '{
          pc_x: 0,
          insn: 0,
          rs1_data: 0, 
          rs2_data: 0, 
          insn_one_hot: 0,
          cycle_status: CYCLE_DIV2USE
        };
    end else begin
        execute_state <= '{
          pc_x: decode_state.pc,
          insn: decode_state.insn,
          rs1_data: rs1_data_decoded,
          rs2_data: rs2_data_decoded,
          insn_one_hot: insn_one_hot,
          cycle_status: decode_state.cycle_status
        };
    end
  end


  // components of the instruction
  wire [6:0] insn_funct7_x;
  wire [4:0] insn_rs2_x;
  wire [4:0] insn_rs1_x;
  wire [2:0] insn_funct3_x;
  wire [4:0] insn_rd_x;
  wire [`OPCODE_SIZE] insn_opcode_x;

  // split R-type instruction - see section 2.2 of RiscV spec
  assign {insn_funct7_x, insn_rs2_x, insn_rs1_x, insn_funct3_x, insn_rd_x, insn_opcode_x} = execute_state.insn;

  // setup for I, S, B & J type instructions
  // I - short immediates and loads
  wire [11:0] imm_i;
  assign imm_i = execute_state.insn[31:20];
  wire [4:0] imm_shamt = execute_state.insn[24:20];

  // S - stores
  wire [11:0] imm_s;
  assign imm_s[11:5] = insn_funct7_x, imm_s[4:0] = insn_rd_x;

  // B - conditionals
  wire [12:0] imm_b;
  assign {imm_b[12], imm_b[10:5]} = insn_funct7_x, {imm_b[4:1], imm_b[11]} = insn_rd_x, imm_b[0] = 1'b0;

  // J - unconditional jumps
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {execute_state.insn[31:12], 1'b0};

  wire [`REG_SIZE] imm_i_sext = {{20{imm_i[11]}}, imm_i[11:0]};
  wire [`REG_SIZE] imm_s_sext = {{20{imm_s[11]}}, imm_s[11:0]};
  wire [`REG_SIZE] imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
  wire [`REG_SIZE] imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};

  // Stores calculated value from execute phase
  logic [`REG_SIZE] rd_data_x;
  logic result_neg;
  logic [63:0] mul_result;
  logic we_x;

  // MX and WX bypass:
  logic[`REG_SIZE] rs1_data_x, rs2_data_x; 
  wire mx_bypass_rs1 = (insn_rs1_x == insn_rd_m) & (insn_rs1_x != 0) & (insn_opcode_m != OpcodeStore);
  wire wx_bypass_rs1 = (insn_rs1_x == insn_rd_w) & (insn_rs1_x != 0);
  wire mx_bypass_rs2 = (insn_rs2_x == insn_rd_m) & (insn_rs2_x != 0) & (insn_opcode_m != OpcodeStore);
  wire wx_bypass_rs2 = (insn_rs2_x == insn_rd_w) & (insn_rs2_x != 0);

  // DIV2USE Stall
  wire div2use_rs1 = (insn_rd_x == insn_rs1_d);
  wire div2use_rs2 = (insn_rd_x == insn_rs2_d);
  wire div_execute = (insn_opcode_x == OpcodeRegReg && execute_state.insn[31:25] == 7'd1) && (execute_state.insn[14:12] == 3'b100 || execute_state.insn[14:12] == 3'b101 || execute_state.insn[14:12] == 3'b110 || execute_state.insn[14:12] == 3'b111);
  wire div_stall_required = (div2use_rs1 || div2use_rs2) && div_execute;

  logic halt_x;

  // WX AND MX BYPASSING LOGIC
  always_comb begin
    rs1_data_x = execute_state.rs1_data;
    rs2_data_x = execute_state.rs2_data;
    if (wx_bypass_rs1 & insn_opcode_w != OpcodeBranch) begin
      rs1_data_x = writeback_state.rd_data;
    end
    if (mx_bypass_rs1 & insn_opcode_m != OpcodeBranch) begin
      rs1_data_x = memory_state.rd_data;
    end 
    if (wx_bypass_rs2 & insn_opcode_w != OpcodeBranch) begin
      rs2_data_x = writeback_state.rd_data;
    end
    if (mx_bypass_rs2 & insn_opcode_m != OpcodeBranch) begin
      rs2_data_x = memory_state.rd_data;
    end
  end
 

  logic [`REG_SIZE] pc_x; // end
  logic [`REG_SIZE] addr_to_dmem_x;

  wire [`REG_SIZE] rs1_add_imm = rs1_data_x + imm_i_sext;
  wire [`REG_SIZE] rs1_add_imm_s = rs1_data_x + imm_s_sext;

  logic [`REG_SIZE] d_dividend, d_divisor, d_remainder, d_quotient;

  divider_unsigned_pipelined divider (
    .clk(clk),
    .rst(rst),
    .i_dividend(d_dividend),
    .i_divisor(d_divisor),
    .o_remainder(d_remainder),
    .o_quotient(d_quotient)
  );


  always_comb begin
    illegal_insn = 1'b0;
    halt_x = 1'b0;
    branch_taken = 1'b0;
    pc_x = 0;
    addr_to_dmem_x = 0;

    case (execute_state.insn_one_hot)
      // fence
      47'h400000000000: begin
         we_x = 1'b0;
         rd_data_x = 'd0;
         halt_x = 1'b0;
      end
      // lui
      47'h200000000000: begin
        rd_data_x = {execute_state.insn[31:12], 12'b0};
        we_x = 1;
      end
      // auipc
      47'h100000000000: begin
        we_x = 1'b1;
        rd_data_x = execute_state.pc_x + {execute_state.insn[31:12], 12'b0};
      end
      // jal
      47'h80000000000: begin
        we_x = 1'b1;
        rd_data_x = execute_state.pc_x + 4;
        pc_x = execute_state.pc_x + imm_j_sext;
        branch_taken = 1'b1;
      end

      // jalr
      47'h40000000000: begin
        we_x = 1'b1;
        rd_data_x = execute_state.pc_x + 4;
        pc_x = (rs1_data_x + imm_i_sext) & 32'hfffffffe;
        branch_taken = 1'b1;
      end
      // beq
      47'h20000000000: begin
        branch_taken = (rs1_data_x == rs2_data_x);
        pc_x = (rs1_data_x == rs2_data_x) ? (execute_state.pc_x + imm_b_sext) : execute_state.pc_x;
        we_x = 0;
      end
      // bne
      47'h10000000000: begin
        branch_taken = (rs1_data_x !== rs2_data_x);
        pc_x = (rs1_data_x !== rs2_data_x) ? (execute_state.pc_x + imm_b_sext) : execute_state.pc_x;
        we_x = 0;
      end
      // blt
      47'h8000000000: begin
        branch_taken = ($signed(rs1_data_x) < $signed(rs2_data_x));
        pc_x = ($signed(rs1_data_x) < $signed(rs2_data_x)) ? (execute_state.pc_x + imm_b_sext) : execute_state.pc_x;
        we_x = 0;
      end
      // bge
      47'h4000000000: begin
        branch_taken = $signed(rs1_data_x) >= $signed(rs2_data_x);
        pc_x = $signed(rs1_data_x) >= $signed(rs2_data_x) ? (execute_state.pc_x + imm_b_sext) : execute_state.pc_x;
        we_x = 0;
      end
      // bltu
      47'h2000000000: begin
        branch_taken = rs1_data_x < rs2_data_x;
        pc_x = (rs1_data_x < rs2_data_x) ? (execute_state.pc_x + imm_b_sext) : execute_state.pc_x;
        we_x = 0;
      end
      // bgeu
      47'h1000000000: begin
        branch_taken = rs1_data_x >= rs2_data_x;
        pc_x = (rs1_data_x >= rs2_data_x) ? (execute_state.pc_x + imm_b_sext) : execute_state.pc_x;
        we_x = 0;
      end
      // lb
      47'h800000000: begin
        we_x = 1'b1; 
        addr_to_dmem_x = ((rs1_data_x + imm_i_sext) >> 2) << 2;
      end
      // lh
      47'h400000000: begin
        we_x = 1'b1; 
        addr_to_dmem_x = ((rs1_data_x + imm_i_sext) >> 2) << 2;
      end
      // lw
      47'h200000000: begin
        we_x = 1'b1; 
        addr_to_dmem_x = ((rs1_data_x + imm_i_sext) >> 2) << 2;
      end
      // lbu
      47'h100000000: begin
        we_x = 1'b1; 
        addr_to_dmem_x = ((rs1_data_x + imm_i_sext) >> 2) << 2;
      end
      // lhu
      47'h80000000: begin
        we_x = 1'b1; 
        addr_to_dmem_x = ((rs1_data_x + imm_i_sext) >> 2) << 2;
      end
      // sb
      47'h40000000: begin
        we_x = 1'b0; 
        addr_to_dmem_x = ((rs1_data_x + imm_s_sext) >> 2) << 2;
      end
      // sh
      47'h20000000: begin
        we_x = 1'b0; 
        addr_to_dmem_x = ((rs1_data_x + imm_s_sext) >> 2) << 2;
      end
      // sw
      47'h10000000: begin
        we_x = 1'b0; 
        addr_to_dmem_x = ((rs1_data_x + imm_s_sext) >> 2) << 2;
      end
      // addi
      47'h8000000: begin
        rd_data_x = rs1_data_x + imm_i_sext;
        we_x = 1;
      end
      // slti
      47'h4000000: begin
        rd_data_x = ($signed(rs1_data_x) < $signed(imm_i_sext)) ? 1 : 0;
        we_x = 1;
      end
      // sltiu
      47'h2000000: begin
        rd_data_x = ($unsigned(rs1_data_x) < $unsigned(imm_i_sext)) ? 1 : 0;
        we_x = 1;
      end
      // xori
      47'h1000000: begin
        rd_data_x = rs1_data_x ^ imm_i_sext;
        we_x = 1;
      end
      // ori
      47'h800000: begin
        rd_data_x = rs1_data_x | imm_i_sext;
        we_x = 1;
      end
      // andi
      47'h400000: begin
        rd_data_x = rs1_data_x & imm_i_sext;
        we_x = 1;
      end
      // slli
      47'h200000: begin
        rd_data_x = rs1_data_x << imm_i[4:0];
        we_x = 1;
      end
      // srli
      47'h100000: begin
        rd_data_x = rs1_data_x >> imm_i[4:0];
        we_x = 1;
      end
      // srai
      47'h80000: begin
        rd_data_x = $signed(rs1_data_x) >>> imm_i[4:0];
        we_x = 1;
      end
      // add
      47'h40000: begin
        rd_data_x = rs1_data_x + rs2_data_x;
        we_x = 1;
      end
      // sub
      47'h20000: begin
        rd_data_x = rs1_data_x + ~rs2_data_x + 1;
        we_x = 1;
      end
      // sll
      47'h10000: begin
        rd_data_x = rs1_data_x << rs2_data_x[4:0];
        we_x = 1;
      end
      // slt
      47'h8000: begin
        rd_data_x = ($signed(rs1_data_x) < $signed(rs2_data_x)) ? 1 : 0;
        we_x = 1;
      end
      // sltu
      47'h4000: begin
        rd_data_x = ($unsigned(rs1_data_x) < $unsigned(rs2_data_x)) ? 1 : 0;
        we_x = 1;
      end
      // xor
      47'h2000: begin
        rd_data_x = rs1_data_x ^ rs2_data_x;
        we_x = 1;
      end
      // srl
      47'h1000: begin
        rd_data_x = rs1_data_x >> rs2_data_x[4:0];
        we_x = 1;
      end
      // sra
      47'h800: begin
        rd_data_x = $signed(rs1_data_x) >>> rs2_data_x[4:0];
        we_x = 1;
      end
      // or
      47'h400: begin
        rd_data_x = rs1_data_x | rs2_data_x;
        we_x = 1;
      end
      // and
      47'h200: begin
        rd_data_x = rs1_data_x & rs2_data_x;
        we_x = 1;
      end
      // mul
      47'h100: begin
        rd_data_x = rs1_data_x * rs2_data_x;
        we_x = 1;
      end
      // mulh
      47'h80: begin
        mul_result = $signed(rs1_data_x) * $signed(rs2_data_x);
        rd_data_x = mul_result[63:32];
        we_x = 1;
      end
      // mulhsu
      47'h40: begin
        result_neg = ($signed(rs1_data_x) < 0) ? 1 : 0;
        mul_result = $signed(rs1_data_x) * $signed({1'b0, rs2_data_x});
        rd_data_x = mul_result[63:32];
        we_x = 1;
      end
      // mulhu
      47'h20: begin
        mul_result = $unsigned(rs1_data_x) * $unsigned(rs2_data_x);
        rd_data_x = mul_result[63:32];
        we_x = 1;
      end
      // div
      47'h10: begin
        we_x = 1'b1;
        d_dividend = $unsigned($signed(rs1_data_x) < 0 ? (~rs1_data_x + 1) : rs1_data_x);
        d_divisor = $unsigned($signed(rs2_data_x) < 0 ? (~rs2_data_x + 1) : rs2_data_x);
      end
      // divu
      47'h8: begin
        we_x = 1'b1;
        d_dividend = $unsigned(rs1_data_x);
        d_divisor = $unsigned(rs2_data_x);
      end
      // rem
      47'h4: begin
        we_x = 1'b1;
        d_dividend = $unsigned($signed(rs1_data_x) < 0 ? (~rs1_data_x + 1) : rs1_data_x);
        d_divisor = $unsigned($signed(rs2_data_x) < 0 ? (~rs2_data_x + 1) : rs2_data_x);
      end
      // remu
      47'h2: begin
        we_x = 1'b1;
        d_dividend = $unsigned(rs1_data_x);
        d_divisor = $unsigned(rs2_data_x);
      end
      // ecall
      47'h1: begin
        halt_x = 1'b1;
      end
    endcase
  end

  wire [(8*32)-1:0] x_disasm;
  Disasm #(
      .PREFIX("X")
  ) disasm_2execute (
      .insn  (execute_state.insn),
      .disasm(x_disasm)
  );

  /*****************************************************/
  /*                    MEMORY STAGE                   */
  /*****************************************************/

  stage_memory_t memory_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      memory_state <= '{
        pc: 0,
        insn: 0,
        rd_data: 0,
        reg_we_m: 0,
        rd: 0,
        halt_m: 0,
        cycle_status: CYCLE_RESET,
        addr_to_dmem_m: 0,
        rs1_data_m: 0,
        rs2_data_m: 0,
        imm_i_sext_m: 0,
        imm_s_sext_m: 0,
        insn_one_hot: 0,
        rs1_add_imm_m: 0,
        rs1_add_imm_s: 0
      };
    end else begin
      begin
        memory_state <= '{
          pc: execute_state.pc_x,
          insn: execute_state.insn,
          rd_data: rd_data_x,
          reg_we_m: we_x,
          rd: insn_rd_x,
          halt_m: halt_x,
          cycle_status: execute_state.cycle_status,
          addr_to_dmem_m: addr_to_dmem_x,
          rs1_data_m: rs1_data_x,
          rs2_data_m: rs2_data_x,
          imm_i_sext_m: imm_i_sext,
          imm_s_sext_m: imm_s_sext,
          insn_one_hot: execute_state.insn_one_hot,
          rs1_add_imm_m: rs1_add_imm,
          rs1_add_imm_s: rs1_add_imm_s
        };
      end
    end
  end

  // components of the instruction
  wire [6:0] insn_funct7_m;
  wire [4:0] insn_rs2_m;
  wire [4:0] insn_rs1_m;
  wire [2:0] insn_funct3_m;
  wire [4:0] insn_rd_m;
  wire [`OPCODE_SIZE] insn_opcode_m;

  // WM Bypass Logic
  wire w_load = insn_opcode_w == OpcodeLoad;
  wire m_store = insn_opcode_m == OpcodeStore;
  wire wm_conflict = (insn_rs2_m == insn_rd_w) & (insn_rs2_m != 0);
  wire wm_bypass = wm_conflict & m_store & w_load;

  // split R-type instruction - see section 2.2 of RiscV spec
  assign {insn_funct7_m, insn_rs2_m, insn_rs1_m, insn_funct3_m, insn_rd_m, insn_opcode_m} = memory_state.insn;
  logic [`REG_SIZE] rd_data_m;
  logic [`REG_SIZE] wm_bypassed_store_data;
  logic result_neg_m;

  // actual code logic starts here
  
  always_comb begin
    addr_to_dmem = memory_state.addr_to_dmem_m;
    wm_bypassed_store_data = (wm_bypass) ? writeback_state.rd_data : memory_state.rs2_data_m;
    rd_data_m = memory_state.rd_data;
    store_data_to_dmem = 'd0;
    store_we_to_dmem = 4'b0000;
    case (memory_state.insn_one_hot)

      // fence
      47'h400000000000: begin
        store_data_to_dmem = 'd0;
        store_we_to_dmem = 4'b0000;
        rd_data_m = 'd0;
      end
      // lb
      47'h800000000: begin
        case(memory_state.rs1_add_imm_m[1:0])
            2'b00: begin
                rd_data_m = {{24{load_data_from_dmem[7]}}, load_data_from_dmem[7:0]};
            end
            2'b01: begin
                rd_data_m = {{24{load_data_from_dmem[15]}}, load_data_from_dmem[15:8]};
            end
            2'b10: begin
                rd_data_m = {{24{load_data_from_dmem[23]}}, load_data_from_dmem[23:16]};
            end
            2'b11: begin
                rd_data_m = {{24{load_data_from_dmem[31]}}, load_data_from_dmem[31:24]};
            end
            default: begin
            end
        endcase
      end
      // lh
      47'h400000000: begin
        case(((memory_state.rs1_add_imm_m) << 30) >> 30)
            32'b00: begin
                rd_data_m = {{16{load_data_from_dmem[15]}}, load_data_from_dmem[15:0]};
            end
            32'b10: begin
                rd_data_m = {{16{load_data_from_dmem[31]}}, load_data_from_dmem[31:16]};
            end
            default: begin
            end
        endcase
      end
      // lw
      47'h200000000: begin
        case(((memory_state.rs1_add_imm_m) << 30) >> 30)
            32'b00: begin
                rd_data_m = load_data_from_dmem;
            end
            default: begin
            end
        endcase
      end
      // lbu
      47'h100000000: begin
        case(((memory_state.rs1_add_imm_m) << 30) >> 30)
            32'b00: begin
                rd_data_m = {24'b0, load_data_from_dmem[7:0]};
            end
            32'b01: begin
                rd_data_m = {24'b0, load_data_from_dmem[15:8]};
            end
            32'b10: begin
                rd_data_m = {24'b0, load_data_from_dmem[23:16]};
            end
            32'b11: begin
                rd_data_m = {24'b0, load_data_from_dmem[31:24]};
            end
            default: begin
            end
        endcase
      end
      // lhu
      47'h80000000: begin
        case((memory_state.rs1_add_imm_m << 30) >> 30)
            32'b00: begin
                rd_data_m = {16'b0, load_data_from_dmem[15:0]};
            end
            32'b10: begin
                rd_data_m = {16'b0, load_data_from_dmem[31:16]};
            end
            default: begin
            end
        endcase
      end
      // sb
      47'h40000000: begin
        case((memory_state.rs1_add_imm_s << 30) >> 30) 
          32'b00: begin
            store_data_to_dmem[7:0] = wm_bypassed_store_data[7:0];
            store_we_to_dmem = 4'b0001;
          end
          32'b01: begin
            store_data_to_dmem[15:8] = wm_bypassed_store_data[7:0];
            store_we_to_dmem = 4'b0010;
          end
          32'b10: begin
            store_data_to_dmem[23:16] = wm_bypassed_store_data[7:0];
            store_we_to_dmem = 4'b0100;
          end
          32'b11: begin
            store_data_to_dmem[31:24] = wm_bypassed_store_data[7:0];
            store_we_to_dmem = 4'b1000;
          end
          default: begin
          end
          endcase
      end

      // sh
      47'h20000000: begin
        case((memory_state.rs1_add_imm_s << 30) >> 30)
            32'b00: begin
                store_data_to_dmem[15:0] = wm_bypassed_store_data[15:0];
                store_we_to_dmem = 4'b0011;
            end
            32'b10: begin
                store_data_to_dmem[31:16] = wm_bypassed_store_data[15:0];
                store_we_to_dmem = 4'b1100;
            end
            default: begin
            end
        endcase
      end
      // sw
      47'h10000000: begin
        store_data_to_dmem = wm_bypassed_store_data;
        store_we_to_dmem = 4'b1111;
      end
      // div
      47'h10: begin
        result_neg_m = ($signed(memory_state.rs1_data_m) < 0) ^ ($signed(memory_state.rs2_data_m) < 0);

        if (memory_state.rs2_data_m == 0) begin
            rd_data_m = 32'hFFFFFFFF;
        end else if (result_neg_m) begin
            rd_data_m = ~d_quotient + 1;
        end else begin
            rd_data_m = d_quotient;
        end
      end
      // divu
      47'h8: begin
        rd_data_m = d_quotient;
      end
      // rem
      47'h4: begin
        result_neg_m = ($signed(memory_state.rs1_data_m) < 0) ? 1 : 0;

        if (memory_state.rs2_data_m == 0) begin
            rd_data_m = $signed(memory_state.rs1_data_m);
        end else if (result_neg_m) begin
            rd_data_m = ~d_remainder + 1;
        end else begin
            rd_data_m = d_remainder;
        end
      end
      // remu
      47'h2: begin
        rd_data_m = d_remainder;
      end
      default: begin
        
      end
    endcase
  end

  wire [(8*32)-1:0] m_disasm;
  Disasm #(
      .PREFIX("M")
  ) disasm_3memory (
      .insn  (memory_state.insn),
      .disasm(m_disasm)
  );

  /*****************************************************/
  /*                  WRITE-BACK STAGE                 */
  /*****************************************************/

  stage_writeback_t writeback_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      writeback_state <= '{
        pc: 0,
        insn: 0,
        rd_data: 0,
        reg_we_w: 0,
        rd: 0,
        halt_w: 0,
        cycle_status: CYCLE_RESET
      };
    end else begin
      begin
        writeback_state <= '{
          pc: memory_state.pc,
          insn: memory_state.insn,
          rd_data: rd_data_m,
          reg_we_w: memory_state.reg_we_m,
          rd: memory_state.rd,
          halt_w: memory_state.halt_m,
          cycle_status: memory_state.cycle_status
        };
      end
    end
  end

  logic we_w = writeback_state.reg_we_w;
  logic [`REG_SIZE] rd_data_w = writeback_state.rd_data;
  assign halt = writeback_state.halt_w;

  // components of the instruction
  wire [6:0] insn_funct7_w;
  wire [4:0] insn_rs2_w;
  wire [4:0] insn_rs1_w;
  wire [2:0] insn_funct3_w;
  wire [4:0] insn_rd_w;
  wire [`OPCODE_SIZE] insn_opcode_w;

  assign trace_writeback_pc = writeback_state.pc;
  assign trace_writeback_insn = writeback_state.insn;
  assign trace_writeback_cycle_status = writeback_state.cycle_status;

  // split R-type instruction - see section 2.2 of RiscV spec
  assign {insn_funct7_w, insn_rs2_w, insn_rs1_w, insn_funct3_w, insn_rd_w, insn_opcode_w} = writeback_state.insn;

  wire [(8*32)-1:0] w_disasm;
  Disasm #(
      .PREFIX("W")
  ) disasm_4writeback (
      .insn  (writeback_state.insn),
      .disasm(w_disasm)
  );
endmodule

module MemorySingleCycle #(
    parameter int NUM_WORDS = 512
) (
    // rst for both imem and dmem
    input wire rst,

    // clock for both imem and dmem. The memory reads/writes on @(negedge clk)
    input wire clk,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] pc_to_imem,

    // the value at memory location pc_to_imem
    output logic [`REG_SIZE] insn_from_imem,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] addr_to_dmem,

    // the value at memory location addr_to_dmem
    output logic [`REG_SIZE] load_data_from_dmem,

    // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
    input wire [`REG_SIZE] store_data_to_dmem,

    // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
    // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
    input wire [3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  logic [`REG_SIZE] mem[NUM_WORDS];

  initial begin
    $readmemh("mem_initial_contents.hex", mem, 0);
  end

  always_comb begin
    // memory addresses should always be 4B-aligned
    assert (pc_to_imem[1:0] == 2'b00);
    assert (addr_to_dmem[1:0] == 2'b00);
  end

  localparam int AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam int AddrLsb = 2;

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      insn_from_imem <= mem[{pc_to_imem[AddrMsb:AddrLsb]}];
    end
  end

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      if (store_we_to_dmem[0]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
      end
      if (store_we_to_dmem[1]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
      end
      if (store_we_to_dmem[2]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
      end
      if (store_we_to_dmem[3]) begin
        mem[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
      end
      // dmem is "read-first": read returns value before the write
      load_data_from_dmem <= mem[{addr_to_dmem[AddrMsb:AddrLsb]}];
    end
  end
endmodule

/* This design has just one clock for both processor and memory. */
module RiscvProcessor (
    input  wire  clk,
    input  wire  rst,
    output logic halt,
    output wire [`REG_SIZE] trace_writeback_pc,
    output wire [`INSN_SIZE] trace_writeback_insn,
    output cycle_status_e trace_writeback_cycle_status
);

  wire [`INSN_SIZE] insn_from_imem;
  wire [`REG_SIZE] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) the_mem (
      .rst                (rst),
      .clk                (clk),
      // imem is read-only
      .pc_to_imem         (pc_to_imem),
      .insn_from_imem     (insn_from_imem),
      // dmem is read-write
      .addr_to_dmem       (mem_data_addr),
      .load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem (mem_data_to_write),
      .store_we_to_dmem   (mem_data_we)
  );

  DatapathPipelined datapath (
      .clk(clk),
      .rst(rst),
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),
      .store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),
      .load_data_from_dmem(mem_data_loaded_value),
      .halt(halt),
      .trace_writeback_pc(trace_writeback_pc),
      .trace_writeback_insn(trace_writeback_insn),
      .trace_writeback_cycle_status(trace_writeback_cycle_status)
  );

endmodule
