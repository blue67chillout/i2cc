// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

module user_proj_example #(
    parameter N=7,Div = 2
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,


    // IOs
    input  [17:0] io_in,
    output [9:0] io_out,
    output [27:0] io_oeb,
);
    wire clk;
    wire rst;
    wire ack_a;
    wire en;
    wire [N-1:0]addr;
    wire rw;       // write || rw=0 ; read || rw = 1
    wire [7:0]din;
    wire scl;
    wire sda;
    wire [7:0]dout 
    
    // IO
    assign io_out[1:0] = {scl,sda};
    assign io_out[9:2] = dout;
    assign {din,addr,ack_a,en,rw} = io_in;
    assign io_oeb = 28'hFFFFC0;


    assign clk =  wb_clk_i;
    assign rst =  wb_rst_i;

    i2c #(
        .N(N),.Div(Div)
    ) i2c(
        .clk(clk),
        .reset(rst),
        .ack_a(ack_a),
        .en(en),
        .rw(rw),
        .addr(addr),
        .din(din),
        .dout(dout),
        .scl(scl),
        .sda(sda)
    );

endmodule


module i2c #(
    parameter N=7,Div = 2
) (
   sda,scl,ack_a,clk,rst,addr,rw,en,din,dout
);
    output sda; //serial data
    output scl; //serial clk
    input ack_a;
    input clk;
    input rst;
    input en;
    input [N-1:0]addr;
    input rw;       // write || rw=0 ; read || rw = 1
    input [7:0]din;
    
    output reg [7:0]dout ;// for read mode

localparam IDLE = 4'd1;
localparam START = 4'd2;
localparam ADDR = 4'd3;
localparam ACK0 = 4'd4; //ack for addr verification and rw
localparam W_DATA = 4'd5;
localparam ACK1 = 4'd6; //ack for successful write
localparam R_DATA = 4'd7;
localparam ACK2 = 4'd8; //ack for successfull read
localparam STOP = 4'd9;

reg [3:0]state;
reg [N:0] addrw; // temp for addr and rw desn't affect that much just saves a state
reg [7:0] saved_data; //temp for din
reg sda_out;  //temp for sda
reg clk_temp = 0;
reg [2:0]count;
reg [2:0]counter = 0;
assign scl = (en == 1)?clk_temp:1'b1;
assign sda = (en == 1)?sda_out:1'b1;


//the implementation is a moore machine which changes states a/c to the i2c protocol


always@(posedge clk) begin
    if(counter == Div-1) begin
        clk_temp <= ~clk_temp;    // clk divider to set relation between
        counter <= 0;           //   sytem clk and scl 
    end
    else counter <= counter + 1;
end

always@(posedge clk_temp or posedge rst ) begin
    if(rst ) begin
        state <= IDLE;
    end
    else begin
        case(state)
            IDLE:  begin
                state <= START;
                addrw <= {addr,rw}; 
            end 

            START: begin
                state <= ADDR;
                count <=3'd7;
                saved_data <= din;
            end

            ADDR: begin
                if (count == 0 && addrw[0] == 0) state <= ACK0;
                else count<= count - 1; 
            end

            ACK0: begin
                if(ack_a == 0 && addrw[0] == 0) begin
                    state <= W_DATA;
                    count <= 3'd7;
                end
                else if(ack_a == 0 && addrw[0] == 1) begin
                    state <= R_DATA;
                    count <= 3'd7;
                end
                else state <= STOP;
            end

            W_DATA: begin
                if(count == 0 )begin
                    state <= ACK1;
                end
                else count <= count - 1;
            end

            R_DATA: begin
                if(count == 0 )begin
                    state <= ACK2;
                end
                else count <= count - 1;
            end

            ACK1: begin
                if(ack_a == 0) begin
                    state <= W_DATA;
                    count <= 3'd7;
                end
                else state <= STOP;
            end

            ACK2: begin
                if(ack_a == 0) begin
                    state <= R_DATA;
                    count <= 3'd7;
                end
                else state <= STOP;
            end

            STOP: begin
                state <= IDLE;
            end
            default:state <= IDLE;

        endcase
    end
    
end

always@(negedge clk_temp or posedge rst ) begin
    if(rst) begin
        sda_out <= 1'b1;
    end
    else begin
        case (state)
            IDLE:sda_out <= 1;
            ADDR:sda_out <= addrw[count];
            W_DATA: sda_out <= saved_data[count];
            R_DATA:dout[count] <= sda_out;
            ACK2:sda_out <= 0;
            default:sda_out <= 1; 
        endcase
    end
end

always@(posedge clk_temp or posedge rst ) begin
    if(rst ) begin
        sda_out <= 1'b1;
    end
    else begin
        case (state)
            START: sda_out <=0;   //still have doubt  about the triggering at this point
            STOP : sda_out <=1;

        endcase
    end
end
endmodule

`default_nettype wire
