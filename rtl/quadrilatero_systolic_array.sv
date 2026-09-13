// Copyright 2024 EPFL
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Author: Danilo Cammarata

/*

TODO:
- handle matrices operations with matrices < MESH_WIDTH based on the configuration CSRs
    - basically you need to inject zeros instead of actual elements
*/

module quadrilatero_systolic_array #(
    parameter  int MESH_WIDTH  = 4                      ,
    parameter  int DATA_WIDTH  = 32                     ,
    parameter  int N_REGS      = 8                      ,
    parameter  int ENABLE_SIMD = 1                      ,
    localparam int N_ROWS      = MESH_WIDTH             ,
    localparam int RLEN        = DATA_WIDTH * MESH_WIDTH,
    parameter FPU = 1
) (
    input  logic                           clk_i               ,
    input  logic                           rst_ni              ,

    output logic                           sa_ready_o          ,
    input  logic                           start_i             ,

    // Only has effect if ENABLE_SIMD == 1
    input  quadrilatero_pkg::sa_ctrl_t       sa_ctrl_i           ,

    input  logic [     $clog2(N_REGS)-1:0] data_reg_i          ,  // data register
    input  logic [     $clog2(N_REGS)-1:0] acc_reg_i           ,  // accumulator register
    input  logic [     $clog2(N_REGS)-1:0] weight_reg_i        ,  // weight register
    input  logic [xif_pkg::X_ID_WIDTH-1:0] id_i                ,  // id of the instruction

    // Weight Read Register Port
    output logic [     $clog2(N_REGS)-1:0] weight_raddr_o      ,
    output logic [     $clog2(N_ROWS)-1:0] weight_rrowaddr_o   ,
    input  logic [               RLEN-1:0] weight_rdata_i      ,
    input  logic                           weight_rdata_valid_i,
    output logic                           weight_rdata_ready_o,
    output logic                           weight_rlast_o      ,

    // Data Read Register Port
    output logic [     $clog2(N_REGS)-1:0] data_raddr_o        ,
    output logic [     $clog2(N_ROWS)-1:0] data_rrowaddr_o     ,
    input  logic [               RLEN-1:0] data_rdata_i        ,
    input  logic                           data_rdata_valid_i  ,
    output logic                           data_rdata_ready_o  ,
    output logic                           data_rlast_o        ,

    // Accumulator Read Register Port
    output logic [     $clog2(N_REGS)-1:0] acc_raddr_o         ,
    output logic [     $clog2(N_ROWS)-1:0] acc_rrowaddr_o      ,
    input  logic [               RLEN-1:0] acc_rdata_i         ,
    input  logic                           acc_rdata_valid_i   ,
    output logic                           acc_rdata_ready_o   ,
    output logic                           acc_rlast_o         ,

    // Accumulator Out Write Register Port
    output logic [     $clog2(N_REGS)-1:0] res_waddr_o         ,
    output logic [     $clog2(N_ROWS)-1:0] res_wrowaddr_o      ,
    output logic [               RLEN-1:0] res_wdata_o         ,
    output logic                           res_we_o            ,
    output logic                           res_wlast_o         ,
    input  logic                           res_wready_i        ,

    // RF Instruction ID
    output logic [xif_pkg::X_ID_WIDTH-1:0] sa_input_id_o       ,
    output logic [xif_pkg::X_ID_WIDTH-1:0] sa_output_id_o      ,

    // Finish 
    output logic                           finished_o          ,
    input  logic                           finished_ack_i      ,
    output logic [xif_pkg::X_ID_WIDTH-1:0] finished_instr_id_o
);

  logic                           ff_active_d        ;
  logic                           ff_active_q        ;
  logic                           fs_active_d        ;
  logic                           fs_active_q        ;
  logic                           dr_active_d        ;
  logic                           dr_active_q        ;
  logic                           set_ff_active      ;
  logic                           rst_ff_active      ;
  logic                           set_fs_active      ;
  logic                           rst_fs_active      ;
  logic                           set_dr_active      ;
  logic                           rst_dr_active      ;
  logic                           valid              ;
  logic                           clear              ;
  logic                           ff_enable          ;
  logic                           fs_enable          ;
  logic                           dr_enable          ;
  logic                           pump               ;
  logic                           stall              ;
  logic                           dep_hazard         ;
  logic [$clog2(MESH_WIDTH)-1 :0] ff_counter_d       ;
  logic [$clog2(MESH_WIDTH)-1 :0] ff_counter_q       ;
  logic [$clog2(MESH_WIDTH)-1 :0] fs_counter_d       ;
  logic [$clog2(MESH_WIDTH)-1 :0] fs_counter_q       ;
  logic [$clog2(MESH_WIDTH)-1 :0] dr_counter_d       ;
  logic [$clog2(MESH_WIDTH)-1 :0] dr_counter_q       ;

  logic [     $clog2(N_REGS)-1:0] data_reg_d         ;  // Data register
  logic [     $clog2(N_REGS)-1:0] data_reg_q         ;  // Data register
  logic [     $clog2(N_REGS)-1:0] acc_reg_d          ;  // Accumulator register -- FF Stage
  logic [     $clog2(N_REGS)-1:0] acc_reg_q          ;  // Accumulator register -- FF Stage
  logic [     $clog2(N_REGS)-1:0] weight_reg_q       ;  // Weight register
  logic [     $clog2(N_REGS)-1:0] weight_reg_d       ;  // Weight register
  quadrilatero_pkg::sa_ctrl_t       sa_ctrl_d          ;
  quadrilatero_pkg::sa_ctrl_t       sa_ctrl_q          ;

  logic [     $clog2(N_REGS)-1:0] dest_reg_q         ;  // Accumulator register -- DR Stage
  logic [     $clog2(N_REGS)-1:0] dest_reg_d         ;  // Accumulator register -- DR Stage

  logic [xif_pkg::X_ID_WIDTH-1:0] id_ff_d            ;
  logic [xif_pkg::X_ID_WIDTH-1:0] id_ff_q            ;
  logic [xif_pkg::X_ID_WIDTH-1:0] id_dr_d            ;
  logic [xif_pkg::X_ID_WIDTH-1:0] id_dr_q            ;

// FF -> DR identity handoff. 
// Stall-safe update to prevent the drain phase from retiring under a stale 
// (previous) instruction ID when FF stalls on the RF scoreboard.
  typedef struct packed {
    logic [xif_pkg::X_ID_WIDTH-1:0] id     ;
    logic [     $clog2(N_REGS)-1:0] acc_reg;
  } sa_ident_t;

  sa_ident_t id_fifo_in    ;
  sa_ident_t id_fifo_out   ;
  logic      id_fifo_push  ;
  logic      id_fifo_pop   ;
  logic      id_fifo_empty ;
  logic      id_fifo_full  ;

  logic                           finished_d         ;
  logic                           finished_q         ;
  logic [xif_pkg::X_ID_WIDTH-1:0] finished_instr_id_d;
  logic [xif_pkg::X_ID_WIDTH-1:0] finished_instr_id_q;
  logic                           mask_req           ;

  quadrilatero_pkg::sa_ctrl_t [MESH_WIDTH-1:0]             sa_ctrl_mesh_skewed;

  logic                 [MESH_WIDTH-1:0][DATA_WIDTH-1:0] data_mesh_skewed   ;
  logic                 [MESH_WIDTH-1:0][DATA_WIDTH-1:0] acc_mesh_skewed    ;
  logic [MESH_WIDTH-1:0][MESH_WIDTH-1:0][DATA_WIDTH-1:0] weight_mesh_skewed ;
  logic                 [MESH_WIDTH-1:0][DATA_WIDTH-1:0] res_mesh_skewed    ;

  //---------------------------------------------------------------------

  always_comb begin: rf_block
    // Weight Read Register Port
    weight_raddr_o       = weight_reg_q              ;
    weight_rrowaddr_o    = ff_counter_q              ;
    weight_rdata_ready_o = ff_active_q &~ mask_req   ;
    weight_rlast_o       = ff_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1);

    // Data Read Register Port
    data_raddr_o         = data_reg_q                ;
    data_rrowaddr_o      = ff_counter_q              ;
    data_rdata_ready_o   = ff_active_q  &~ mask_req  ;
    data_rlast_o         = ff_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1);

    // Accumulator Read Register Port
    acc_raddr_o          = acc_reg_q                 ;
    acc_rrowaddr_o       = ff_counter_q              ;
    acc_rdata_ready_o    = ff_active_q &~ mask_req   ;
    acc_rlast_o          = ff_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1);

    // Accumulator Out Write Register Port
    res_waddr_o         = dest_reg_q                ;
    res_wrowaddr_o      = dr_counter_q              ;
    res_we_o            = dr_active_q  &~ mask_req  ;
    res_wlast_o         = dr_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1);
  end

  always_comb begin: next_value

    // Configuration
    data_reg_d    = (set_ff_active) ? data_reg_i    : data_reg_q   ;
    acc_reg_d     = (set_ff_active) ? acc_reg_i     : acc_reg_q    ;
    weight_reg_d  = (set_ff_active) ? weight_reg_i  : weight_reg_q ;
    sa_ctrl_d     = (set_ff_active) ? sa_ctrl_i     : sa_ctrl_q    ;

    dest_reg_d    = (set_dr_active) ? id_fifo_out.acc_reg : dest_reg_q;

    id_ff_d       = (set_ff_active) ? id_i          : id_ff_q      ;
    id_dr_d       = (set_dr_active) ? id_fifo_out.id : id_dr_q     ;

    // Finished
    finished_d          = (res_wready_i && res_wlast_o) ? 1'b1 :
                          (finished_ack_i             ) ? 1'b0 : finished_q;

    finished_instr_id_d = (res_wready_i && res_wlast_o) ? id_dr_q :
                          (finished_ack_i             ) ? '0      : finished_instr_id_q; 

    // Counters
    ff_counter_d = (ff_enable && ff_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1)) ? '0               :
                   (ff_enable                              ) ? ff_counter_q + 1 : ff_counter_q;
                   
    fs_counter_d = (clear                                  ) ||
                   (fs_enable && fs_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1))    ? '0               :
                   (fs_enable                              )    ? fs_counter_q + 1 : fs_counter_q;

    dr_counter_d = (clear                                  ) ||
                   (dr_enable && dr_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1))    ? '0               :
                   (dr_enable                              )    ? dr_counter_q + 1 : dr_counter_q;

    // Active signals
    ff_active_d = set_ff_active ? 1'b1 :
                  rst_ff_active ? 1'b0 : ff_active_q;

    fs_active_d = set_fs_active ? 1'b1 :
                  rst_fs_active ? 1'b0 : fs_active_q;

    dr_active_d = set_dr_active ? 1'b1 :
                  rst_dr_active ? 1'b0 : dr_active_q;
  end

  always_comb begin: ctrl_block
    valid = weight_rdata_valid_i & data_rdata_valid_i & acc_rdata_valid_i;
    clear = ~ff_active_q & ~fs_active_q & ~dr_active_q;

    // Global Pump Synchronization
    //
    // The mesh, skewers, and weight double-buffer share a SINGLE pump. An FF lap 
    // strictly requires MESH_WIDTH consecutive pumps to maintain spatial alignment 
    // between propagating data and weight fetches.
    //
    // 1. Array Freeze Rule: If FF stalls waiting for an operand, the ENTIRE array 
    //    must freeze. FS and DR phases are forbidden from pumping independently.
    // 2. Deadlock Safety: Freezing DR is safe. Internal RAW hazards are handled 
    //    by `dep_hazard`, so FF only stalls on external LSU/perm-unit writes.
    stall     = ff_active_q & ~valid                ;
    ff_enable = ff_active_q &  valid                ;
    fs_enable = fs_active_q & ~stall                ;
    dr_enable = dr_active_q & ~stall                ;

    set_ff_active = ff_counter_d=='0 & start_i                                                                                          ;
    set_fs_active = fs_counter_d=='0 & ff_counter_d=='0                                & ff_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1);
    set_dr_active = dr_counter_d=='0 & fs_counter_d==$clog2(MESH_WIDTH)'(MESH_WIDTH-1) & fs_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-2);

    rst_ff_active = ff_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1) & ff_counter_d=='0                                      ;
    rst_fs_active = fs_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1) & fs_counter_d=='0 & ff_counter_d=='0 & ff_counter_q=='0;
    rst_dr_active = dr_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1) & dr_counter_d=='0 & fs_counter_d=='0 & fs_counter_q=='0;

    pump     = ff_enable | fs_enable | dr_enable                              ;
    mask_req = (dr_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1)) & finished_q & ~finished_ack_i;

    // one push per instruction leaving FF, one pop per drain lap starting
    id_fifo_push        = ff_enable & (ff_counter_q==$clog2(MESH_WIDTH)'(MESH_WIDTH-1));
    id_fifo_pop         = set_dr_active                                                ;
    id_fifo_in.id       = id_ff_q                                                      ;
    id_fifo_in.acc_reg  = acc_reg_q                                                    ;

    // Start-time RAW guard: a new instruction whose source register is the destination of an
    // instruction still inside the array (FF, FS or DR) would stall in FF waiting for the DR
    // write-back, and with the shared pump frozen that write-back could never happen.  Such an
    // instruction waits in the issue queue until the array is empty (exactly what N<=2 does today).
    dep_hazard = (ff_active_q    & (data_reg_i==acc_reg_q           | weight_reg_i==acc_reg_q           | acc_reg_i==acc_reg_q          ))
               | (~id_fifo_empty & (data_reg_i==id_fifo_out.acc_reg | weight_reg_i==id_fifo_out.acc_reg | acc_reg_i==id_fifo_out.acc_reg))
               | (dr_active_q    & (data_reg_i==dest_reg_q          | weight_reg_i==dest_reg_q          | acc_reg_i==dest_reg_q         ));
  end

  fifo_v3 #(
      .FALL_THROUGH(0         ),
      .DEPTH       (2         ),
      .dtype       (sa_ident_t)
  ) id_fifo_i (
      .clk_i                   ,
      .rst_ni                  ,
      .flush_i   (1'b0        ),
      .testmode_i(1'b0        ),
      .usage_o   (/* unused */),
      .full_o    (id_fifo_full ),
      .empty_o   (id_fifo_empty),
      .data_i    (id_fifo_in  ),
      .push_i    (id_fifo_push),
      .data_o    (id_fifo_out ),
      .pop_i     (id_fifo_pop )
  );

  quadrilatero_skewer #(
      .MESH_WIDTH(MESH_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
  ) skewer_inst_data (
      .clk_i                    ,
      .rst_ni                   ,
      .pump_i (pump            ),
      .data_i (data_rdata_i    ),
      .data_o (data_mesh_skewed)
  );

  quadrilatero_skewer #(
      .MESH_WIDTH(MESH_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
  ) skewer_inst_acc (
      .clk_i                   ,
      .rst_ni                  ,
      .pump_i (pump           ),
      .data_i (acc_rdata_i    ),
      .data_o (acc_mesh_skewed)
  );

  quadrilatero_skewer #(
      .MESH_WIDTH(MESH_WIDTH),
      .DATA_WIDTH(4)
  ) skewer_inst_ctrl (
      .clk_i                           ,
      .rst_ni                          ,
      .pump_i (pump                   ),
      .data_i ({MESH_WIDTH{sa_ctrl_q}}),
      .data_o (sa_ctrl_mesh_skewed    )
  );

  quadrilatero_wl_stage #(
      .MESH_WIDTH(MESH_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
  ) weight_inst (
    .clk_i                                           ,
    .rst_ni                                          ,

    .ff_counter             (ff_counter_q           ),
    .clear_i                (clear                  ),
    .pump_i                 (pump                   ),
    .weight_rdata_valid_i                            ,
    
    // Weight Data 
    .weight_rdata_i                                  ,
    .weight_rdata_o         (weight_mesh_skewed     ) 
  );

  quadrilatero_mesh #(
      .MESH_WIDTH (MESH_WIDTH ),
      .ENABLE_SIMD(ENABLE_SIMD),
      .FPU        (FPU        )
  ) mesh_inst (
      .clk_i,
      .rst_ni,

      .pump_i         (pump                   ),
      .sa_ctrl_i      (sa_ctrl_mesh_skewed    ),

      .data_i         (data_mesh_skewed       ),
      .acc_i          (acc_mesh_skewed        ),
      .weight_i       (weight_mesh_skewed     ),
      .acc_o          (res_mesh_skewed        )
  );

  quadrilatero_deskewer #(
      .MESH_WIDTH(MESH_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
  ) deskewer_inst_acc (
      .clk_i                   ,
      .rst_ni                  ,
      .pump_i (pump           ),
      .data_i (res_mesh_skewed),
      .data_o (res_wdata_o    )
  );

  always_ff @(posedge clk_i or negedge rst_ni) begin: seq_block
    if (!rst_ni) begin
      ff_counter_q        <= '0;
      fs_counter_q        <= '0;
      dr_counter_q        <= '0;
      ff_active_q         <= '0;
      fs_active_q         <= '0;
      dr_active_q         <= '0;
      data_reg_q          <= '0;
      acc_reg_q           <= '0;
      weight_reg_q        <= '0;
      sa_ctrl_q           <= '0;
      dest_reg_q          <= '0;
      id_ff_q             <= '0;
      id_dr_q             <= '0;
      finished_q          <= '0;
      finished_instr_id_q <= '0;
    end else begin
      ff_counter_q        <= ff_counter_d        ;
      fs_counter_q        <= fs_counter_d        ;
      dr_counter_q        <= dr_counter_d        ;
      ff_active_q         <= ff_active_d         ;
      fs_active_q         <= fs_active_d         ;
      dr_active_q         <= dr_active_d         ;
      data_reg_q          <= data_reg_d          ;
      acc_reg_q           <= acc_reg_d           ;
      weight_reg_q        <= weight_reg_d        ;
      sa_ctrl_q           <= sa_ctrl_d           ;
      dest_reg_q          <= dest_reg_d          ;
      id_ff_q             <= id_ff_d             ;
      id_dr_q             <= id_dr_d             ;
      finished_q          <= finished_d          ;
      finished_instr_id_q <= finished_instr_id_d ;
    end
  end
 
  assign sa_ready_o          = (ff_counter_d=='0) & ((ff_active_q &~ ff_counter_q=='0) | (~ff_active_q & ~fs_active_q & ~dr_active_q)) & ~dep_hazard;
  assign sa_input_id_o       = id_ff_q            ;
  assign sa_output_id_o      = id_dr_q            ;
  assign finished_o          = finished_q         ;
  assign finished_instr_id_o = finished_instr_id_q;

  // --------------------------------------------------------------------

  // Assertions
`ifdef XSIM
  assert property (@(posedge clk_i) disable iff (!rst_ni) id_fifo_pop |-> !id_fifo_empty)
  else $error("[systolic_array] drain lap started with no identity queued");
  assert property (@(posedge clk_i) disable iff (!rst_ni) id_fifo_push |-> !id_fifo_full)
  else $error("[systolic_array] identity queue overflow");
  // The invariant the shared pump relies on: with FS/DR frozen alongside FF, an FF lap always
  // ends on an FS lap boundary, so the identity handoff can never be skipped and at most one
  // instruction is ever queued between FF and DR.
  assert property (@(posedge clk_i) disable iff (!rst_ni) id_fifo_push |-> set_fs_active)
  else $error("[systolic_array] FF finished off the FS lap boundary (pump/FF misaligned)");
  assert property (@(posedge clk_i) disable iff (!rst_ni) id_fifo_push |-> id_fifo_empty)
  else $error("[systolic_array] identity queue held an entry when a second one was pushed");
  // FS/DR may only advance while FF is not waiting for an operand.
  assert property (@(posedge clk_i) disable iff (!rst_ni) stall |-> !pump)
  else $error("[systolic_array] pump ran while FF was stalled");
`endif

  if (MESH_WIDTH < 2) begin
    $error(
        "[systolic_array] MESH_WIDTH must be at least 2.\n"
    );
  end
endmodule
