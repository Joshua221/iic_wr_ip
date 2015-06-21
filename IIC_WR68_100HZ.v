/***************************************************************************/
/* File: IIC_WR68_100HZ.v                                                  */                    
/***************************************************************************/
 
//寄存器操作模块 IIC_WR68_100HZ.v
`timescale 10ns / 10ns 
module IIC_WR68_100HZ(
    // Clcok Input
    input CLOCK, RESET,
    // Avalon-MM Slave：为nios读写内部寄存器所用 
    input chipselect,
    input [7:0] address,
    input write,
    input [31:0] writedata,  
    input read,
    output [31:0] readdata,
     
    // Custom interface
    output SCL,
    inout SDA
);
  
//相关的速率还有时序参数声明&相关的伪函数声明
    parameter FCLK = 10'd500, FHALF = 10'd250, FQUARTER = 10'd125; //(1/400E+3)/(1/50E+6)
    parameter THIGH = 10'd200, TLOW = 10'd235, TR = 10'd50, TF = 10'd15;
    parameter THD_STA = 10'd200, TSU_STA = 10'd235, TSU_STO = 10'd200;
    parameter FF_Write1 = 5'd7;
    parameter FF_Write2 = 5'd9, RDFUNC = 5'd19;
  
    //Internal signals: 需要被软件读写的寄存器写在最前面（要为其编写register map）
    reg [1:0]iCall;  // 10——写任务 ； 01——读任务 ； 00——无任务  表示读写器模块的工作状态
    reg [31:0]iAddr;  // 操作地址（被控芯片）
    reg [31:0]iData;  // 要写内容
    reg [31:0]Read;   // 读任务时从SDA上读取的数据
  
    //其他内部变量,不用提供接口
     
    reg [31:0]read_data_register; //readdata寄存器
    reg [7:0]Data; // 有效字节
    reg [4:0]i;    // 状态机变量
    reg [4:0]Go;   // 主线状态
    reg [9:0]C1;   // 周期计数器
  
    reg rSCL,rSDA; // IIC时钟&信号两线
    reg isAck, isDone, isQ;  // IIC应答信号、当前读写任务的完成信号、数据线方向控制位
  
   //重要寄存器需要有能响应nios2的读写请求（read/write,chipselect,address）的avalon接口
    //Nodes used for address decoding 地址译码Flag
    wire iAddr_reg_selected, iData_reg_selected, iCall_reg_selected;
 
    //Nodes for determining if a valid write occurred to a specific address
    wire write_to_iAddr, write_to_iData, write_to_iCall;
 
    //Nodes for determining if a valid read occurred to a specific address
    wire read_from_iAddr, read_from_iData, read_from_iCall;
 
    //Nodes used to determine if a valid access has occurred
    wire valid_write, valid_read;
 
    //address decode 对应及寄存器的定义顺序
    assign iCall_reg_selected = !address[1] & !address[0];  //address 00
    assign iAddr_reg_selected = !address[1] &  address[0];  //address 01
    assign iData_reg_selected =  address[1] & !address[0];  //address 10
     
    //determine if a vaild transaction was initiated 
    assign valid_write = chipselect & write;      
    assign valid_read = chipselect & read;
 
    //determine if a write occurred to a specific address
    assign write_to_iAddr = valid_write & iAddr_reg_selected;
    assign write_to_iData = valid_write & iData_reg_selected;
    assign write_to_iCall = valid_write & iCall_reg_selected;
 
    //determine if a read occurred to a specific address
    assign read_from_iAddr = valid_read & iAddr_reg_selected;
    assign read_from_iData = valid_read & iData_reg_selected;
    assign read_from_iCall = valid_read & iCall_reg_selected;
 
    always@(posedge CLOCK or negedge RESET)
    begin
        if(!RESET)begin //Async Reset
            iAddr <= 32'h0000_0000;
        end
            else begin
        if(write_to_iAddr) begin
            iAddr <= writedata;    
        end
        else begin
            iAddr <= iAddr;  
        end
        end
    end
 
 
  
    always@(posedge CLOCK or negedge RESET)
    begin
        if(!RESET)begin //Async Reset
            iData <= 32'h0000_0000;
        end
            else begin
        if(write_to_iData) begin
            iData <= writedata;    
        end
        else begin
            iData <= iData;  
        end
        end
    end
 
    
    always@(posedge CLOCK or negedge RESET)
    begin
        if(!RESET)begin //Async Reset
                iCall <= 2'b00;
        end
        else begin
        if(write_to_iCall)begin
            iCall[1:0] <= writedata[1:0];
        end
        else begin
            iCall <= iCall;
        end
        end
    end
 
    //Read Data Bus Mux
    always@(read_from_iAddr or read_from_iData or read_from_iCall or iAddr or iData or iCall)
    begin
    if(read_from_iAddr) begin
        read_data_register = iAddr;
    end
    else if(read_from_iData) begin
        read_data_register = iData;
    end
    else if(read_from_iCall) begin
        read_data_register = {30'd0,iCall};
    end
    else begin
        read_data_register = 32'h0000_0000;
    end
    end
 
//主任务状态机
    always @ ( posedge CLOCK or negedge RESET )
       if( !RESET )
          begin
              { i,Go } <= { 5'd0,5'd0 };//状态机复位
              C1 <= 10'd0; //一个变量的初始化、复位工作都做全在一个always里
              Data <= 8'd0;
              Read <= 8'd0;
              { rSCL,rSDA,isAck,isDone,isQ } <= 5'b11101;
          end
       else if( iCall == 2'b10)//C语言通过修改iCall值，立刻执行写任务（发送iAddr和iData现有内容），此处与不能有chipselect
        /*使能写操作：主状态机的驱动只与IP核内部寄存器iCall的值有关（由C语言间接启动）
            0:125->1:1->7~14:8*125->15:125->16:1->
                   2:1->7~14:8*125->15:125->16:1->
                   3:1->7~14:8*125->15:125->16:1->
                   4:156->
                   5:1->
                   6:1
          共耗时：3664clk */
          case( i )
             0: // Call:产生起始位
                 begin
                      isQ = 1;
                      rSCL <= 1'b1;
                        
                      if( C1 == 0 ) rSDA <= 1'b1; //维持45clk
                      else if( C1 == (TR + THIGH) ) rSDA <= 1'b0;
   
                      if( C1 == (FCLK) -1)  //起始信号的产生耗时1个SCL周期（对应125个输入clk）
                          begin 
                             C1 <= 10'd0;   //为下一个状态重置时钟
                             i <= i + 1'b1; //准备进入下一个状态 
                          end
                      else C1 <= C1 + 1'b1; //又过一个clk
   
                 end
  
             1: // Write Device Addr:写入设备地址，并且调用伪函数
                 begin 
                      Data <= {4'b1010, 3'b000, 1'b0}; 
                      i <= FF_Write1; 
                      Go <= i + 1'b1; //回到主线后应该从下一个状态继续
                 end
   
             2: // Wirte Word Addr:写入数据地址，并且调用伪函数
                 begin 
                      Data <= iAddr[7:0]; 
                      i <= FF_Write1; 
                      Go <= i + 1'b1; 
                 end
   
             3: // Write Data:写入数据，并且调用伪函数
                 begin 
                      Data <= iData[7:0]; 
                      i <= FF_Write1; 
                      Go <= i + 1'b1; 
                 end
  
  
             4: // Stop：产生结束位
                 begin            
                      isQ = 1'b1;
  
                      if( C1 == 0 ) rSCL <= 1'b0;
                      else if( C1 == FQUARTER ) rSCL <= 1'b1; 
   
                      if( C1 == 0 ) rSDA <= 1'b0;
                      else if( C1 == (FQUARTER + TR + TSU_STO ) ) rSDA <= 1'b1;
   
                      if( C1 == (FQUARTER + FCLK) -1 ) //严格的结束位操作需要1又1/4个scl周期
                           begin 
                                C1 <= 10'd0; 
                                i <= i + 1'b1; 
                           end
                      else C1 <= C1 + 1'b1; 
  
                 end
  
              5://完成信号
                 begin 
                      isDone <= 1'b1; //发送一次
                      i <= i + 1'b1; 
                 end
   
              6: 
  
               begin 
                      isDone <= 1'b0; 
                      i <= 5'd0; //为下一个命令复位状态机
                 end
   
//function
              7,8,9,10,11,12,13,14://写一个字节的伪函数，占据8个scl操作周期
                 begin 
                      isQ = 1'b1;
                      rSDA <= Data[14-i]; //从最高位（左）传起，一个操作周期只传1bit
  
                      if( C1 == 0 ) rSCL <= 1'b0;
                      else if( C1 == (TF + TLOW) ) rSCL <= 1'b1; 
   
                      if( C1 == FCLK -1 ) 
                          begin 
                              C1 <= 10'd0; 
                              i <= i + 1'b1; 
                          end
                      else C1 <= C1 + 1'b1;
                  end
   
               15: // waiting for acknowledge
                  begin
                       isQ = 1'b0;
                       if( C1 == FHALF ) isAck <= SDA;
  
                       if( C1 == 0 ) rSCL <= 1'b0;
                       else if( C1 == FHALF ) rSCL <= 1'b1;
  
                       if( C1 == FCLK -1 ) 
                       begin 
                            C1 <= 10'd0; 
                            i <= i + 1'b1; 
                       end
                       else C1 <= C1 + 1'b1; 
                  end
                  
               16://判断应答位，应答成功则回归主线步骤（继续发送其他数据，或者结束），失败则重新来过
  
                   if( isAck != 0 ) i <= 5'd0;
                   else i <= Go; 
               // end function
           endcase
   
       else if( iCall == 2'b01 ) //使能读操作
          case( i )
               0: // Start
                 begin
                       isQ = 1;  
                       rSCL <= 1'b1;
  
                       if( C1 == 0 ) rSDA <= 1'b1;  
                       else if( C1 == (TR + THIGH) ) rSDA <= 1'b0;
  
                       if( C1 == FCLK -1 ) 
                       begin 
                            C1 <= 10'd0; 
                            i <= i + 1'b1; 
                       end
                       else C1 <= C1 + 1'b1; 
                 end
   
               1: // Write Device Addr
                 begin 
                       Data <= {4'b1010, 3'b000, 1'b0}; 
                       i <= FF_Write2; 
                       Go <= i + 1'b1; 
                 end
  
               2: // Wirte Word Addr
                 begin 
                       Data <= iAddr[7:0]; 
                       i <= FF_Write2; 
                       Go <= i + 1'b1; 
                 end
                 
               3: // Start again
                 begin
                       isQ = 1'b1;
  
                       if( C1 == 0 ) rSCL <= 1'b0;
                       else if( C1 == FQUARTER ) rSCL <= 1'b1;
                       else if( C1 == (FQUARTER + TR + TSU_STA + THD_STA + TF) ) rSCL <= 1'b0;
   
                       if( C1 == 0 ) rSDA <= 1'b0; 
                       else if( C1 == FQUARTER ) rSDA <= 1'b1;
                       else if( C1 == ( FQUARTER + TR + THIGH) ) rSDA <= 1'b0;
  
                       if( C1 == (FQUARTER + FCLK + FQUARTER) -1 ) begin C1 <= 10'd0; i <= i + 1'b1; end
                       else C1 <= C1 + 1'b1;
                 end
   
               4: // Write Device Addr ( Read )
                  begin 
                        Data <= {4'b1010, 3'b000, 1'b1}; 
                        i <= 5'd9; 
                        Go <= i + 1'b1; 
                  end
   
               5: // Read Data：读取一个字节
                  begin 
                        Read <= 8'd0; 
                        i <= RDFUNC; 
                        Go <= i + 1'b1; end
   
               6: // Stop
                  begin
                        isQ = 1'b1;
                        if( C1 == 0 ) rSCL <= 1'b0;
                        else if( C1 == FQUARTER ) rSCL <= 1'b1; 
                           
                        if( C1 == 0 ) rSDA <= 1'b0;
                        else if( C1 == (FQUARTER + TR + TSU_STO) ) rSDA <= 1'b1;
                        if( C1 == (FCLK + FQUARTER) -1 ) begin C1 <= 10'd0; i <= i + 1'b1; end
                        else C1 <= C1 + 1'b1; 
                  end
   
               7://产生完成信号
                  begin 
                        isDone <= 1'b1; 
                        i <= i + 1'b1;
                  end
   
               8: 
                  begin 
                        isDone <= 1'b0;
                        i <= 5'd0; 
                  end
//function
               9,10,11,12,13,14,15,16://写一字节的伪函数
                  begin
                        isQ = 1'b1;
                        rSDA <= Data[16-i];
  
                        if( C1 == 0 ) rSCL <= 1'b0;
                        else if( C1 == (TF + TLOW) ) rSCL <= 1'b1; 
   
                        if( C1 == FCLK -1 ) begin C1 <= 10'd0; i <= i + 1'b1; end
                        else C1 <= C1 + 1'b1;
                  end
  
                 
               17: // waiting for acknowledge
                  begin
                        isQ = 1'b0;
                        if( C1 == FHALF ) isAck <= SDA;
                        if( C1 == 0 ) rSCL <= 1'b0;
                        else if( C1 == FHALF ) rSCL <= 1'b1;
                        if( C1 == FCLK -1 ) begin C1 <= 10'd0; i <= i + 1'b1; end
                        else C1 <= C1 + 1'b1; 
                  end
  
                          
               18://判断应答位
                  if( isAck != 0 ) i <= 5'd0;
                  else i <= Go;
  
               19,20,21,22,23,24,25,26: // Read：读取一字节的伪函数
                  begin
                         isQ = 1'b0;
                         if( C1 == FHALF ) Read[26-i] <= SDA;
                         if( C1 == 0 ) rSCL <= 1'b0;
                         else if( C1 == FHALF ) rSCL <= 1'b1; 
                         if( C1 == FCLK -1 ) begin C1 <= 10'd0; i <= i + 1'b1; end
                         else C1 <= C1 + 1'b1;
                  end
  
               27: // no acknowledge：无视应答位
                  begin
                      isQ = 1'b1;
                      if( C1 == 0 ) rSCL <= 1'b0;
                      else if( C1 == FHALF ) rSCL <= 1'b1;
                      if( C1 == FCLK -1 ) begin C1 <= 10'd0; i <= Go; end
  
                      else C1 <= C1 + 1'b1; 
                  end
          // end fucntion
          endcase
   
    assign SCL = rSCL;
    assign SDA = isQ ? rSDA : 1'bz;
    assign readdata = read_data_register;
  
endmodule
