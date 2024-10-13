module fir
  #(  parameter pADDR_WIDTH = 12,
      parameter pDATA_WIDTH = 32,
      parameter Tape_Num    = 11
   )
   (
     output  wire                     awready,
     output  wire                     wready,
     input   wire                     awvalid,
     input   wire [(pADDR_WIDTH-1):0] awaddr,
     input   wire                     wvalid,
     input   wire [(pDATA_WIDTH-1):0] wdata,
     output  wire                     arready,
     input   wire                     rready,
     input   wire                     arvalid,
     input   wire [(pADDR_WIDTH-1):0] araddr,
     output  wire                     rvalid,
     output  wire [(pDATA_WIDTH-1):0] rdata,
     input   wire                     ss_tvalid,
     input   wire [(pDATA_WIDTH-1):0] ss_tdata,
     input   wire                     ss_tlast,
     output  wire                     ss_tready,
     input   wire                     sm_tready,
     output  wire                     sm_tvalid,
     output  wire [(pDATA_WIDTH-1):0] sm_tdata,
     output  wire                     sm_tlast,

     // bram for tap RAM
     output  wire [3:0]               tap_WE,
     output  wire                     tap_EN,
     output  wire [(pDATA_WIDTH-1):0] tap_Di,
     output  wire [(pADDR_WIDTH-1):0] tap_A,
     input   wire [(pDATA_WIDTH-1):0] tap_Do,

     // bram for data RAM
     output  wire [3:0]               data_WE,
     output  wire                     data_EN,
     output  wire [(pDATA_WIDTH-1):0] data_Di,
     output  wire [(pADDR_WIDTH-1):0] data_A,
     input   wire [(pDATA_WIDTH-1):0] data_Do,

     input   wire                     axis_clk,
     input   wire                     axis_rst_n
   );


  reg [(pADDR_WIDTH-1):0] tap_A_w;
  reg [(pDATA_WIDTH-1):0] str_data;
  reg signed[4:0] tap_cnt;
  reg str_last;
  wire ap_idle, ap_done, ap_start;
  wire str_valid;
  wire op_ready;    //fir ready

  //==================================================================================//
  //                                    STATE MACHINE                                 //
  //==================================================================================//
  localparam S_IDLE = 3'd0;
  localparam S_START = 3'd1;
  localparam S_OP = 3'd2;
  localparam S_LITE = 3'd3;
  localparam S_DONE = 3'd4;
  reg [3-1:0] state, nxt_state;


  always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
      state <= S_IDLE;
    else
      state <= nxt_state;

  always@(*)
  case(state)
    S_IDLE:
      if(ap_start)
        nxt_state = S_START;
      else
        nxt_state = S_IDLE;
    S_START:
      if(str_valid)
        nxt_state = S_OP;
      else
        nxt_state = S_START;
    S_OP:
      if(tap_cnt == 10 & str_last)
        nxt_state = S_LITE;
      else if(tap_cnt == 10 & ~str_last)
        nxt_state = S_START;
      else
        nxt_state = S_OP;
    S_LITE:     // TB checking ap_done
      if(rvalid)
        nxt_state = S_DONE;
      else
        nxt_state =  S_LITE;
    S_DONE:
      if(rvalid)
        nxt_state = S_IDLE;
      else
        nxt_state = S_DONE;
    default:
      nxt_state = S_IDLE;
  endcase

  //==================================================================================//
  //                                       AXI-Lite                                   //
  //==================================================================================//

  localparam LITE_idle = 3'd0;
  localparam LITE_READY = 3'd1;   //  read
  localparam LITE_RREQ = 3'd2;
  localparam LITE_READ = 3'd3;

  reg [2:0] lite_state, lite_nxt_state;
  reg arready_r, wready_r;          // read/write
  reg [1:0] lite_req;
  reg [(pADDR_WIDTH-1):0] config_A;
  reg [(pDATA_WIDTH-1):0] w_data;
  wire [(pDATA_WIDTH-1):0] r_data;

  assign awready = (awvalid) ? 1 : 0;
  assign wready =  (awvalid) ? wready_r : 0;
  assign rdata = (lite_state == LITE_READ) ? r_data: 0;
  assign arready = arready_r;
  assign rvalid = (lite_state == LITE_READ) ? 1 : 0;

  always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
      lite_state <= 3'd0;
    else
      lite_state <= lite_nxt_state;

  always@(*)
  begin
    case(lite_state)
      LITE_idle:
        if(arvalid)
          lite_nxt_state = LITE_READY;
        else
          lite_nxt_state = LITE_idle;

      LITE_READY:
        if(arready && arvalid)
          lite_nxt_state = LITE_RREQ;
        else
          lite_nxt_state = LITE_READY;
      LITE_RREQ:
        if(rready)
          lite_nxt_state = LITE_READ;
        else
          lite_nxt_state = LITE_RREQ;
      LITE_READ:
        lite_nxt_state = LITE_idle;
      default:
        lite_nxt_state = 3'd0;
    endcase
  end

  //address for data
  always@(posedge axis_clk or negedge axis_rst_n)
  begin
    if(~axis_rst_n)
    begin
      config_A <= 0;
      lite_req <= 0;
      w_data <= 0;
      arready_r <= 0;
      wready_r <= 0;
    end
    else
    begin
      case(lite_state)
        LITE_idle:
        begin
          config_A <= (awready & wready) ? awaddr : config_A; // write
          lite_req <= (awready & wready) ? 2'd1 : lite_req;   // write
          w_data <= (wready) ? wdata : w_data;
          arready_r <= (arvalid) ? 1 : 0;
          wready_r <= (awvalid) ? 1 : wready_r;
        end
        LITE_READY:
        begin
          w_data <= 0;
          arready_r <= 0;
          config_A <= (arready) ? araddr : config_A;
          lite_req <= (arready && arvalid) ? 2'd2 : lite_req;
        end
        LITE_RREQ:
        begin
          w_data <= 0;
          arready_r <= 0;
        end
        LITE_READ:
        begin
          w_data <= 0;
          arready_r <= 0;
        end
        default:
        begin
          w_data <= 0;
          arready_r <= 0;
          wready_r <= 0;
          config_A <= 12'd1; // addr not being used
          lite_req <= 2'd0;  // no operation
        end
      endcase
    end
  end

  //==================================================================================//
  //                      Configuration Register Access Protocol                      //
  //==================================================================================//
  wire [(pADDR_WIDTH-1):0] data_ram_A;
  wire [(pADDR_WIDTH-1):0] tap_ram_A;
  wire w_en;  // tap_ram 、configeration
  reg [11:0] data_cnt;


  // block level protocal
  assign ap_start = (state == S_IDLE && awaddr == 12'd0 && wdata[0] == 1 && config_A < 12'h10 && lite_req == 2'b01) ? 1:0;
  assign ap_done = ( (sm_tvalid && sm_tlast) || state == S_DONE ) ? 1 : 0;
  assign ap_idle = (state == S_IDLE) ? 1 : 0;

  assign w_en = (lite_req == 2'b01) ? 1 : 0; // write enable
  assign r_data = (araddr[7:0] != 8'd0) ? tap_Do : {ap_idle, ap_done, ap_start};
  // 0x20-FF: Tap parameter
  assign tap_WE = (state == S_IDLE && (config_A >= 12'h20 || config_A == 12'h10)) ? {4{w_en}} : 0;
  assign tap_Di = (state == S_IDLE) ? w_data : 0;
  assign tap_A = tap_A_w;
  assign tap_EN = 1;
  // Data parameter
  assign data_ram_A = (data_cnt < tap_cnt) ? (data_cnt - tap_cnt + 5'd11) << 2 : (data_cnt - tap_cnt) << 2;
  assign data_A = (state == S_IDLE ) ? tap_A : data_ram_A;
  assign data_Di = (state == S_IDLE ) ? 0 : str_data;
  assign data_WE = (state == S_IDLE ) ? tap_WE : (state == S_START ) ? 1 : 0;
  assign data_EN = 1'b1;

  always@(*)
  begin
    case(state)
      S_IDLE:
        tap_A_w = (config_A >= 12'h20) ? config_A - 12'h20 : 12'h10 << 2; //address for tap
      S_START:
        tap_A_w = tap_ram_A;
      S_OP:
        tap_A_w = tap_ram_A;
      S_LITE:
        tap_A_w = tap_ram_A;
      S_DONE:
        tap_A_w = 0;      //clean ap_done, set ap_idle
      default:
        tap_A_w = 0;
    endcase
  end

  //==================================================================================//
  //                                    AXI-Stream_in                                 //
  //==================================================================================//
  localparam STR_IN_IDLE = 1'b0;
  localparam STR_IN_STAR = 1'b1;
  reg [2:0] str_in_state, str_in_nxt_state;
  reg tready_r, str_valid_r;
  wire str_valid_w;

  assign ss_tready = tready_r;
  assign str_valid = str_valid_r;
  assign str_valid_w = (str_in_state == STR_IN_IDLE) ? ss_tready : ss_tready & ss_tvalid;

  always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
      str_in_state <= STR_IN_IDLE;
    else
      str_in_state <= str_in_nxt_state;

  always@(*)
  case(str_in_state)
    STR_IN_IDLE:
      if(ap_start)
        str_in_nxt_state = STR_IN_STAR;
      else
        str_in_nxt_state = STR_IN_IDLE;
    STR_IN_STAR:
      if(ss_tready && ss_tlast)
        str_in_nxt_state = STR_IN_IDLE;
      else
        str_in_nxt_state = STR_IN_STAR;
    default:
      str_in_nxt_state = STR_IN_IDLE;
  endcase

  always@(*)
  begin
    if(str_in_state == STR_IN_IDLE)
      tready_r = ap_start;
    else if(str_in_state == STR_IN_STAR)
      tready_r = op_ready & sm_tready;
    else
      tready_r = 1'b0;
  end

  always@(posedge axis_clk or negedge axis_rst_n)
  begin
    if(~axis_rst_n)
      str_data <= 0;
    else if(str_in_state == STR_IN_IDLE)
      str_data <= (ss_tready && ss_tvalid) ? ss_tdata : str_data;
    else if(str_in_state == STR_IN_STAR)
      str_data <= (ss_tready && ss_tvalid) ? ss_tdata : str_data;
    else
      str_data <= str_data;
  end

  always@(posedge axis_clk or negedge axis_rst_n)
  begin
    if(~axis_rst_n)
    begin
      str_valid_r <= 1'b0;
      str_last <= 0;
    end
    else
    begin
      str_valid_r <= str_valid_w;
      str_last <= (ss_tready) ? ss_tlast : 0;
    end
  end

  //==================================================================================//
  //                                    AXI-Stream_out                                //
  //==================================================================================//
  localparam STR_OUT_IDLE = 1'b0;
  localparam STR_OUTPUT   = 1'b1;
  reg str_out_state, str_out_nxt_state;
  reg [(pDATA_WIDTH-1):0] ff_tdata;

  assign sm_tdata  = (str_out_state == STR_OUTPUT) ? ff_tdata : 0;
  assign sm_tlast  = (str_out_state == STR_OUTPUT) ? (tap_cnt == 10 && str_last && tap_cnt == 10) : 0;
  assign sm_tvalid = (str_out_state == STR_OUTPUT) ? 1 : 0;

  always@(posedge axis_clk or negedge axis_rst_n)
  begin
    if(~axis_rst_n)
      str_out_state <= STR_OUT_IDLE;
    else
      str_out_state <= str_out_nxt_state;
  end

  always@(*)
  begin
    case(str_out_state)
      STR_OUT_IDLE:
        if(tap_cnt == 10)
          str_out_nxt_state = STR_OUTPUT;
        else
          str_out_nxt_state = STR_OUT_IDLE;
      STR_OUTPUT:
        if(sm_tready)
          str_out_nxt_state = STR_OUT_IDLE;
        else
          str_out_nxt_state = STR_OUTPUT;
      default:
        str_out_nxt_state = STR_OUTPUT;
    endcase
  end

  //==================================================================================//
  //                                      FIR Engine                                  //
  //==================================================================================//

  wire signed [(pDATA_WIDTH-1):0] x, h, mul;
  wire [(pDATA_WIDTH-1):0] y_w;
  reg [(pDATA_WIDTH-1):0] y_r;

  assign x = data_Do;   //x[t]
  assign h = tap_Do;
  assign mul = h * x;
  assign y_w = (state == S_OP) ? mul + y_r : 0;
  assign op_ready = (state == S_OP) ? (tap_cnt == 10) & ~str_valid : (state == S_START) ? ~str_valid: 1'b0 ;
  assign tap_ram_A = (tap_cnt << 2);

  always@(posedge axis_clk or negedge axis_rst_n)
  begin
    if(~axis_rst_n)
    begin
      tap_cnt <= 0;
    end
    else if(state == S_IDLE)
    begin
      tap_cnt <= 0;
    end
    else if(state == S_START)
    begin
      tap_cnt <= (str_valid) ? tap_cnt + 5'd1 : tap_cnt;
    end
    else if(state == S_OP)
    begin
      tap_cnt <= (tap_cnt != 10) ? tap_cnt + 5'd1 : 5'd0;
    end
    else
    begin
      tap_cnt <= 5'd0;
    end
  end

  //data_ram
  always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
      data_cnt <= 5'd0;
    else if(tap_cnt == 10 && data_cnt < 5'd10)
      data_cnt <= data_cnt + 5'd1;
    else if(tap_cnt == 10)
      data_cnt <= 5'd0;
    else
      data_cnt <= data_cnt;

  always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
      y_r <= 0;
    else
      y_r <= y_w;

  always@(posedge axis_clk or negedge axis_rst_n)
  begin
    if(~axis_rst_n)
      ff_tdata <= 0;
    else if(tap_cnt == 10)
      ff_tdata <= mul + y_r;
    else if(sm_tready)
      ff_tdata <= 0;
    else
      ff_tdata <= ff_tdata;
  end

endmodule
