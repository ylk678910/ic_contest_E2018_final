module TPA(clk, reset_n,
           SCL, SDA,
           cfg_req, cfg_rdy, cfg_cmd, cfg_addr, cfg_wdata, cfg_rdata);
input 		clk;
input 		reset_n;
// Two-Wire Protocol slave interface
input 		SCL;
inout		SDA;

// Register Protocal Master interface
input		cfg_req;
output		cfg_rdy;
input		cfg_cmd;
input	[7:0]	cfg_addr;
input	[15:0]	cfg_wdata;
output	[15:0]  cfg_rdata;

reg	[15:0] Register_Spaces	[0:255];

// ===== Coding your RTL below here =================================
localparam WRITE = 1'b1,	//RPM, TWP cmd
           READ  = 1'b0;
localparam TRUE  = 1'b1,
           FALSE = 1'b0;

reg SDA_en, SDA_o;
assign SDA = (SDA_en) ? SDA_o : 1'bz;
wire SDA_i = SDA;

//RPM declare
reg RPM_cmd;
reg RPM_start_flg;
reg [16-1 :0] RPM_reg_buf;
reg RPM_got_cmd_flg;
wire RPM_start_signal;
wire RPM_end_flg;
wire [16-1:0] RPM_addr;
wire RPM_w_done_flg;
wire RPM_r_done_flg;

//TWP declare
localparam 	TAR_IDLE  = 2'd0,
            TAR_ING   = 2'd1,
            TAR_S     = 2'd2,
            TAR_DATA  = 2'd3;
reg TWP_cmd;
reg TWP_start_flg;
reg TWP_got_cmd_flg;
reg [5-1 : 0] TWP_timer;
reg [1:0] TWP_TAR_flg;
reg [8-1 :0] TWP_addr;
reg [16-1 :0] TWP_reg_buf;
wire TWP_start_signal;
wire TWP_wdata_flg;
wire TWP_w_done_flg;
wire TWP_r_done_flg;
wire TWP_got_addr_flg;
wire TWP_rdata_flg;
wire TWP_TAR_signal;
wire TWP_end_flg;

//Other declare
reg overwrite_detect_flg;       //Overwrite detect.
reg overwrite_all_write_flg;    //Both commands are WRITE.
reg overwrite_flg;              //TWP writes before RPM writes.
wire overwrite_not_write_signal, overwrite_all_write_signal;
wire overwrite_same_addr_signal, overwrite_diff_addr_signal;

//RPM defined
assign cfg_rdy = RPM_start_flg;
assign cfg_rdata = RPM_reg_buf;
assign RPM_start_signal = cfg_req;
assign RPM_addr = cfg_addr;
assign RPM_w_done_flg = RPM_got_cmd_flg && RPM_cmd==WRITE && !overwrite_detect_flg;
assign RPM_r_done_flg = RPM_got_cmd_flg && RPM_cmd==READ;
assign RPM_end_flg = RPM_w_done_flg || RPM_r_done_flg;

//TWP defined
assign TWP_start_signal = !SDA_i;
assign TWP_wdata_flg = TWP_timer>5'd7;
assign TWP_w_done_flg = (TWP_cmd==WRITE) && TWP_timer==5'd24;
assign TWP_got_addr_flg = TWP_wdata_flg;
assign TWP_rdata_flg = TWP_timer==5'd27;
assign TWP_TAR_signal = TWP_timer==5'd8;
assign TWP_r_done_flg = (TWP_cmd==READ) && TWP_rdata_flg;
assign TWP_end_flg = (TWP_cmd==WRITE) ? TWP_w_done_flg : TWP_r_done_flg;

//Other defined
assign overwrite_not_write_signal = (TWP_got_cmd_flg && TWP_cmd!=WRITE) || RPM_cmd!=WRITE; //When TWP started and command is not WRITE or RPM command is not WRITE.
assign overwrite_all_write_signal = (TWP_got_cmd_flg && TWP_cmd==WRITE) && RPM_cmd==WRITE; //When TWP started and command is WRITE and RPM command is also WRITE.
assign overwrite_same_addr_signal = TWP_got_addr_flg && (TWP_addr==RPM_addr);
assign overwrite_diff_addr_signal = TWP_got_addr_flg && (TWP_addr!=RPM_addr);


//Register control
reg [15:0] reg_w, reg_r;
reg [15:0] reg_w_queue;
reg [7:0] reg_r_queue_addr;
reg [7:0] reg_addr_w, reg_addr_r;
reg reg_queue_flg;
wire write_data_same_time_flg = (TWP_w_done_flg && !overwrite_flg) && RPM_w_done_flg;
wire read_data_same_time_flg = TWP_r_done_flg && RPM_r_done_flg;
wire reg_read_flg = reg_queue_flg || TWP_r_done_flg || RPM_r_done_flg;
wire reg_write_flg = reg_queue_flg || (TWP_w_done_flg && !overwrite_flg) || RPM_w_done_flg;

always @(posedge SCL or negedge reset_n) begin
    if(!reset_n) begin
        reg_queue_flg <= FALSE;
    end
    else begin
        if(reg_read_flg) begin
            reg_r_queue_addr <= TWP_addr;
        end
        if(write_data_same_time_flg) begin
            reg_w_queue <= TWP_reg_buf;
        end
        if(reg_write_flg) begin
            Register_Spaces[reg_addr_w] <= reg_w;
        end
        if(reg_queue_flg) begin
            reg_queue_flg <= FALSE;
        end
        else if(write_data_same_time_flg || read_data_same_time_flg) begin
            reg_queue_flg <= TRUE;
        end
    end
end
always @(*) begin
    if(RPM_r_done_flg) begin
        reg_addr_r = RPM_addr;
    end
    else if(reg_queue_flg) begin
        reg_addr_r = reg_r_queue_addr;
    end
    else begin
        reg_addr_r = TWP_addr;
    end
end

always @(*) begin
    reg_r = Register_Spaces[reg_addr_r];
end

always @(*) begin
    if(RPM_w_done_flg) begin
        reg_addr_w = RPM_addr;
        reg_w = cfg_wdata;
    end
    else if(reg_queue_flg) begin
        reg_addr_w = reg_r_queue_addr;
        reg_w = reg_w_queue;
    end
    else begin //if(TWP_w_done_flg && !overwrite_flg || reg_queue_flg)
        reg_addr_w = TWP_addr;
        reg_w = TWP_reg_buf;
    end
end

//Overwrite
always @(posedge clk or negedge reset_n) begin
    if(!reset_n) begin
        overwrite_detect_flg <= FALSE;
    end
    else  begin
        if(TWP_end_flg) begin
            overwrite_detect_flg <= FALSE;
        end
        else if(overwrite_detect_flg && overwrite_not_write_signal) begin
            overwrite_detect_flg <= FALSE;
        end
        else if(overwrite_all_write_flg && overwrite_diff_addr_signal) begin
            overwrite_detect_flg <= FALSE;
        end
        else if(!RPM_start_flg && RPM_start_signal) begin
            if (TWP_start_flg || TWP_start_signal) begin //TWP before RPM or at the same time
                overwrite_detect_flg <= TRUE;
            end
        end
    end
end

always @(posedge clk or negedge reset_n) begin
    if(!reset_n) begin
        overwrite_all_write_flg <= FALSE;
    end
    else  begin
        if(!overwrite_detect_flg) begin
            overwrite_all_write_flg <= FALSE;
        end
        else if(!overwrite_all_write_signal) begin
            overwrite_all_write_flg <= TRUE;
        end
    end
end

always @(posedge clk or negedge reset_n) begin
    if(!reset_n) begin
        overwrite_flg <= FALSE;
    end
    else  begin
        if(!overwrite_detect_flg) begin
            overwrite_flg <= FALSE;
        end
        else if(overwrite_all_write_flg) begin
            if(overwrite_same_addr_signal) begin
                overwrite_flg <= TRUE;
            end
        end
    end
end

//RPM
always @(posedge clk or negedge reset_n) begin
    if(!reset_n) begin
        RPM_start_flg <= FALSE;
        RPM_got_cmd_flg <= FALSE;
        RPM_cmd <= READ;
    end
    else begin
        RPM_got_cmd_flg <= RPM_start_flg;
        if(RPM_end_flg) begin //End cmd
            RPM_start_flg <= FALSE;
        end
        else if(RPM_start_signal) begin
            RPM_start_flg <= TRUE;
        end
        else begin
            RPM_start_flg <= RPM_start_flg;
        end
        if(RPM_start_signal) begin
            RPM_cmd <= cfg_cmd;
        end
    end
end

always @(*) begin
    RPM_reg_buf = reg_r;
end

//TWP

always @(posedge SCL or negedge reset_n) begin
    if(!reset_n) begin
        TWP_cmd <= READ;
    end
    else if(TWP_start_flg && !TWP_got_cmd_flg) begin
        TWP_cmd <= SDA_i;
    end
end

always @(posedge SCL or negedge reset_n) begin
    if(!reset_n) begin
        TWP_start_flg <= FALSE;
        TWP_addr <= 8'd0;
        TWP_reg_buf <= 16'd0;
        TWP_timer <= 5'd0;
        TWP_got_cmd_flg <= FALSE;
        SDA_en <= FALSE;
        SDA_o <= 1'b0;
    end
    else if(TWP_end_flg)  begin
        TWP_start_flg <= FALSE;
        TWP_got_cmd_flg <= FALSE;
        TWP_addr <= 8'd0;
        TWP_reg_buf <= 16'd0;
        TWP_timer <= 5'd0;
        SDA_en <= FALSE;
        SDA_o <= 1'b0;
    end
    else begin
        if(TWP_start_signal) begin
            TWP_start_flg <= TRUE;
        end
        case(TWP_TAR_flg)
            TAR_ING: begin
                SDA_en <= TRUE;
                SDA_o <= 1'b1;
            end
            TAR_S: begin
                SDA_en <= TRUE;
                SDA_o <= 1'b0;
            end
            TAR_DATA: begin
                SDA_en <= TRUE;
                SDA_o <= reg_r[TWP_timer-5'd11];
            end
            default begin
                SDA_en <= FALSE;
                SDA_o <= 1'b0;
            end
        endcase
        TWP_got_cmd_flg <= TWP_start_flg;
        if(TWP_wdata_flg) begin
            TWP_reg_buf[15]   <= SDA_i;
            TWP_reg_buf[14:0] <= TWP_reg_buf[15:1];
        end
        if(TWP_got_cmd_flg && !TWP_got_addr_flg) begin
            TWP_addr[7]   <= SDA_i;
            TWP_addr[6:0] <= TWP_addr[7:1];
        end
        if(TWP_got_cmd_flg) begin
            TWP_timer <= TWP_timer + 5'd1;
        end
    end
end

always @(posedge SCL or negedge reset_n) begin
    if(!reset_n) begin
        TWP_TAR_flg <= TAR_IDLE;
    end
    else begin
        case(TWP_TAR_flg)
            TAR_IDLE: begin
                if(TWP_cmd==READ && TWP_TAR_signal) begin
                    TWP_TAR_flg <= TAR_ING;
                end
                else begin
                    TWP_TAR_flg <= TAR_IDLE;
                end
            end
            TAR_ING, TAR_S: begin
                TWP_TAR_flg <= TWP_TAR_flg + 3'd1;
            end
            TAR_DATA: begin
                if(TWP_end_flg)
                    TWP_TAR_flg <= TAR_IDLE;
                else
                    TWP_TAR_flg <= TAR_DATA;
            end
            default begin
                TWP_TAR_flg <= TAR_IDLE;
            end
        endcase
    end
end


endmodule
