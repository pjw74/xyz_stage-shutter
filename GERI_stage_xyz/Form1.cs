using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

using System.Net;
using System.Net.Sockets;
using System.Collections;
using System.Threading;
using System.IO.Ports;

using System.Text.RegularExpressions;

using System.IO;
using System.Diagnostics;

using Thorlabs.MotionControl.DeviceManagerCLI;
using Thorlabs.MotionControl.KCube.SolenoidCLI;

using Thorlabs.MotionControl.IntegratedStepperMotorsCLI;
using Thorlabs.MotionControl.GenericMotorCLI;
using Thorlabs.MotionControl.GenericMotorCLI.ControlParameters;
using Thorlabs.MotionControl.GenericMotorCLI.AdvancedMotor;
using Thorlabs.MotionControl.GenericMotorCLI.Settings;



namespace GERI_stage_xyz
{
    public partial class Form1 : Form
    {


        KCubeSolenoid device1;
        KCubeSolenoid device2;
        KCubeSolenoid device3;


        string serialNo1 = "68250406";
        string serialNo2 = "68250445";
        string serialNo3 = "68250435";

        CycleSettings settings = new CycleSettings();

        uint signal1, signal2, signal3;


        Socket sock;

        byte dummy = 0xff;
        byte stx = 0x02;
        byte etx = 0x03;
        byte ACK = 0x06;
        byte nak = 0x15;
        byte rst = 0x12;

        string x_position, y_position, z_position, w_position;
        string error_result_st;

        string spd_str = "0000";

        string ltn_stra1 = "00000.0000";
        string ltn_stra2 = "00000.0000";

        string status = "";

        string ltn_stra5 = "    0.000 ";//일의 자리 수일 때         

        char[] x_pos = new char[10];
        char[] y_pos = new char[10];
        char[] z_pos = new char[10];
        char[] w_pos = new char[10];

        char[] error_result = new char[30];

        double x_start, y_start, z_start;
        double x_pitch_size, y_pitch_size, z_pitch_size;

        double x_stage_point, y_stage_point, z_stage_point;

        int x_count;
        int y_count;
        int count_num;

        decimal x_complete, y_complete, z_complete, w_complete;
        double w_stage_point = 0;

        //double x_point_db, y_point_db;
        double z_up_db, z_down_db;

        //double[] long_point = new double[4];
        //double[] short_point = new double[2];

        SerialPort _serialPort = new SerialPort();

        int open_time_sc10 = 0;

        public Form1()
        {
            InitializeComponent();

            string[] PortNames = SerialPort.GetPortNames();  // 포트 검색.
            foreach (string portnumber in PortNames)
            {
                comboBox1.Items.Add(portnumber);          // 검색한 포트를 콤보박스에 입력. 
            }

        }



        public static byte[] Combine(byte[] first, byte[] second) //byte 결합하는 함수에 관한 부분
        {
            return first.Concat(second).ToArray();
        }

        public byte lrc_cal(byte[] data)  //명령어 LRC 계산하는 부분
        {

            //byte XOR 연산
            byte lrc = dummy;

            for (int n = 0; n < data.Length; n++)
            {
                lrc = (byte)(lrc ^ data[n]);
            }

            if (lrc == 0)
            {
                lrc = etx;
            }

            return lrc;
        }


        private void stg_con_Click(object sender, EventArgs e)
        {
            sock = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            IPAddress ip = IPAddress.Parse("192.168.1.203");//인자값 : 서버측 IP         
            IPEndPoint endPoint = new IPEndPoint(ip, 20000);//인자값 : IPAddress,포트번호

            while (sock.Connected == false)
            {
                sock.Connect(endPoint);
            }

            if (sock.Connected == true)
            {
                textBox1.Text = ("XYZ Stage Connected");
                stg_con.Enabled = false;
            }
        }


        private void stg_dis_Click(object sender, EventArgs e)
        {
            sock.Close();
            textBox1.Text = ("Stage Disconnected");

            if (sock.Connected == false)
            {
                stg_con.Enabled = true;
            }
        }


        private void send_function(byte[] msg)
        {
            byte[] bytes = new byte[50];

            byte[] ack = new byte[] { ACK };
            byte[] header = new byte[] { stx, dummy };

            msg = Combine(header, msg);

            if (sock.Available > 0) // here we clean up the current queue
            {
                sock.Receive(bytes);
            }

            sock.Send(msg);

            while (sock.Available == 0) // wait for the controller response
            {
                Thread.Sleep(100);
            }

            sock.Receive(bytes); // after receiving the data, we should check the LRC if possible
                                 //string status = Encoding.UTF8.GetString(bytes);

            if (bytes.Contains<byte>(nak) || bytes.Contains<byte>(rst) == true)
            {
                sock.Send(msg);
            }
            else
            {
                sock.Send(ack);
            }

            status = Encoding.UTF8.GetString(bytes);

            //msg.Initialize();
        }

        private void send_position(byte[] msg) //좌표 수신
        {

            byte[] bytes = new byte[50];

            byte[] ack = new byte[] { ACK };
            byte[] header = new byte[] { stx, dummy };

            msg = Combine(header, msg);

            if (sock.Available > 0) // here we clean up the current queue
            {
                sock.Receive(bytes);
                //sock1.Receive(bytes1);
            }

            sock.Send(msg);

            while (sock.Available == 0) // wait for the controller response
            {
                Thread.Sleep(100);
            }

            sock.Receive(bytes); // after receiving the data, we should check the LRC if possible

            if (bytes.Contains<byte>(nak) || bytes.Contains<byte>(rst) == true)
            {
                sock.Send(msg);
            }
            else
            {
                sock.Send(ack);
            }

            status = Encoding.UTF8.GetString(bytes);

            status.CopyTo(3, x_pos, 0, 7);
            status.CopyTo(13, y_pos, 0, 7);
            status.CopyTo(23, z_pos, 0, 7);

            //int val = Convert.ToInt32(x_pos[8]);
            //x_pos[8] = Convert.ToChar(48);//Convert.ToChar(val - 1);
            //int x_pos_abs = Convert.ToInt16(x_pos);

            x_position = new string(x_pos);//비교를 위해 초기값 저장
            x_position = x_position.Trim();

            decimal x_pos_abs = Convert.ToDecimal(x_position);
            x_pos_abs = Math.Abs(x_pos_abs);
            x_position = Convert.ToString(x_pos_abs);

            y_position = new string(y_pos);//비교를 위해 초기값 저장
            y_position = y_position.Trim();

            decimal y_pos_abs = Convert.ToDecimal(y_position);
            y_pos_abs = Math.Abs(y_pos_abs);
            y_position = Convert.ToString(y_pos_abs);

            z_position = new string(z_pos);
            z_position = z_position.Trim();

            decimal z_pos_abs = Convert.ToDecimal(z_position);
            z_pos_abs = Math.Abs(z_pos_abs);
            z_position = Convert.ToString(z_pos_abs);

            msg.Initialize();
            bytes.Initialize();

            /*
            decimal x_st = Convert.ToDecimal(x_stage_point);
            string x_st_fn = x_st.ToString(ltn_stra5);
            x_st_fn = x_st_fn.Trim();

            decimal y_st = Convert.ToDecimal(y_stage_point);
            string y_st_fn = y_st.ToString(ltn_stra5);
            y_st_fn = y_st_fn.Trim();

            status = Encoding.UTF8.GetString(bytes1);

            status.CopyTo(3, x_pos, 0, 10);
            status.CopyTo(13, y_pos, 0, 10);

            int val = Convert.ToInt32(x_pos[8]);
            x_pos[8] = Convert.ToChar(val - 1);

            x_position = new string(x_pos);//비교를 위해 초기값 저장
                                           //x_position = Math.Abs(x_position);

            x_position = x_position.Trim();

            y_position = new string(y_pos);//비교를 위해 초기값 저장
            y_position = y_position.Trim();


            decimal x_st = Convert.ToDecimal(x_stage_point);
            string x_st_fn = x_st.ToString(ltn_stra5);
            x_st_fn = x_st_fn.Trim();

            decimal y_st = Convert.ToDecimal(y_stage_point);
            string y_st_fn = y_st.ToString(ltn_stra5);
            y_st_fn = y_st_fn.Trim();

            while (((x_position.Equals(x_st_fn)) && (y_position.Equals(y_st_fn))) == false)
            {
                sock1.Send(make_msg);

                while (sock1.Available == 0) // wait for the controller response
                {
                    Thread.Sleep(10);
                }

                sock1.Receive(bytes1); // after receiving the data, we should check the LRC if possible

                if (bytes1.Contains<byte>(nak) || bytes1.Contains<byte>(rst) == true)
                {
                    sock1.Send(make_msg);
                }
                else
                {
                    sock1.Send(ack1);
                }
                string status1 = Encoding.UTF8.GetString(bytes1);


                status1.CopyTo(3, x_pos, 0, 10);
                status1.CopyTo(13, y_pos, 0, 10);

                x_position = new string(x_pos); //"  100.000 "
                x_position = x_position.Trim();

                y_position = new string(y_pos);
                y_position = y_position.Trim();

                if (((x_position.Equals(x_st_fn)) && (y_position.Equals(y_st_fn))) == true)
                {
                    break;
                }
            }
            msg.Initialize();
            bytes.Initialize();
            */
        }

        private void send_error(byte[] msg) //좌표 수신
        {

            byte[] bytes = new byte[50];

            byte[] ack = new byte[] { ACK };
            byte[] header = new byte[] { stx, dummy };

            msg = Combine(header, msg);

            if (sock.Available > 0) // here we clean up the current queue
            {
                sock.Receive(bytes);
                //sock1.Receive(bytes1);
            }

            sock.Send(msg);

            while (sock.Available == 0) // wait for the controller response
            {
                Thread.Sleep(100);
            }

            sock.Receive(bytes); // after receiving the data, we should check the LRC if possible

            if (bytes.Contains<byte>(nak) || bytes.Contains<byte>(rst) == true)
            {
                sock.Send(msg);
            }
            else
            {
                sock.Send(ack);
            }

            status = Encoding.UTF8.GetString(bytes);

            status.CopyTo(3, error_result, 0, 30);
            //status.CopyTo(13, y_pos, 0, 10);
            //status.CopyTo(23, z_pos, 0, 10);

            error_result_st = new string(error_result);//비교를 위해 초기값 저장

            msg.Initialize();
            bytes.Initialize();
        }

        public byte[] speed(string channel, string spd)
        {
            byte[] command = Encoding.UTF8.GetBytes("CB");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] speed_set = Encoding.UTF8.GetBytes(spd);
            byte[] make_msg = Combine(command, channel_ba);
            make_msg = Combine(make_msg, speed_set);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }

        private void x_left_Click(object sender, EventArgs e)
        {
            decimal speed_1 = spd_value.Value;
            string spd = speed_1.ToString(spd_str);

            var comm_spd = speed("0", spd);
            send_function(comm_spd);

            Thread.Sleep(100);

            var comm = jog_start("0", "0", "0");
            send_function(comm);
        }

        private void x_right_Click(object sender, EventArgs e)
        {
            decimal speed_1 = spd_value.Value;
            string spd = speed_1.ToString(spd_str);

            var comm_spd = speed("0", spd);
            send_function(comm_spd);

            Thread.Sleep(100);

            var comm = jog_start("0", "0", "1");
            send_function(comm);
        }

        private async void y_left_Click(object sender, EventArgs e)
        {
            decimal speed_1 = spd_value.Value; //단축 left
            string spd = speed_1.ToString(spd_str);
            var comm_spd = speed("0", spd);
            send_function(comm_spd);

            Thread.Sleep(100);

            var task1 = Task.Run(() =>
            {
                var comm = jog_start("0", "1", "0");
                send_function(comm);
            });
            await task1;
        }

        private void y_right_Click(object sender, EventArgs e)
        {
            decimal speed_1 = spd_value.Value;
            string spd = speed_1.ToString(spd_str);
            var comm_spd = speed("0", spd);
            send_function(comm_spd);

            Thread.Sleep(100);

            var comm = jog_start("0", "1", "1");
            send_function(comm);
        }

        private void z_up_Click(object sender, EventArgs e)
        {
            decimal speed_1 = spd_value.Value;
            string spd = speed_1.ToString(spd_str);
            var comm_spd = speed("0", spd);
            send_function(comm_spd);

            Thread.Sleep(100);

            var comm = jog_start("0", "2", "1");
            send_function(comm);
        }

        private void z_down_Click(object sender, EventArgs e)
        {
            decimal speed_1 = spd_value.Value;
            string spd = speed_1.ToString(spd_str);
            var comm_spd = speed("0", spd);
            send_function(comm_spd);

            Thread.Sleep(100);

            var comm = jog_start("0", "2", "0");
            send_function(comm);
        }

        public byte[] jog_start(string channel, string axis, string pm)
        {
            byte[] command = Encoding.UTF8.GetBytes("BE");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] axis_type = Encoding.UTF8.GetBytes(axis);
            byte[] direc_type = Encoding.UTF8.GetBytes(pm);
            byte[] motion_type = Encoding.UTF8.GetBytes("0");
            byte[] make_msg = Combine(command, channel_ba);
            make_msg = Combine(make_msg, axis_type);
            make_msg = Combine(make_msg, direc_type);
            make_msg = Combine(make_msg, motion_type);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }

        private void posi_chk_Click(object sender, EventArgs e)
        {
            var comm = posi_check_robot("0");
            send_position(comm);
            // Compare_string(x_stage_point, y_stage_point);
            //Task.Run(() => SetText());
            this.Invoke(new Action(SetText));
        }

        public void SetText()
        {
            lb_x_position.Text = x_position;
            lb_y_position.Text = y_position;
            lb_z_position.Text = z_position;
        }

        public byte[] posi_check_robot(string channel) //robot_position chk
        {
            byte[] command = Encoding.UTF8.GetBytes("AC");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] data_type = Encoding.UTF8.GetBytes("2");
            byte[] make_msg = Combine(command, channel_ba);
            make_msg = Combine(make_msg, data_type);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }

        private void rebot_controller_Click(object sender, EventArgs e)
        {
            var comm = servo_off("0");
            send_function(comm);

            Thread.Sleep(1000);

            var comm1 = rebot_contr();
            send_function(comm1);

            Thread.Sleep(2000);
            //Socket.Disconnect(sock);

            sock.Close();

            sock = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            IPAddress ip = IPAddress.Parse("192.168.1.203");//인자값 : 서버측 IP         
            IPEndPoint endPoint = new IPEndPoint(ip, 20000);//인자값 : IPAddress,포트번호

            while (sock.Connected == false)
            {
                sock.Connect(endPoint);
            }

            if (sock.Connected == true)
            {
                textBox1.Text = ("XYZ Stage Reconnected");
                stg_con.Enabled = false;
            }
            //stg_con.Enabled = true;
        }


        public byte[] rebot_contr()
        {
            byte[] command = Encoding.UTF8.GetBytes("CJ");

            byte lrc = lrc_cal(command);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            command = Combine(command, etx_ba);
            command = Combine(command, lrc_ba);

            return command;
        }

        private void sv_on_Click(object sender, EventArgs e)
        {
            var comm = servo_on("0");
            send_function(comm);
        }

        public byte[] servo_on(string channel)
        {
            byte[] command = Encoding.UTF8.GetBytes("DB");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] data_type = Encoding.UTF8.GetBytes("1");
            byte[] make_msg = Combine(command, channel_ba);
            make_msg = Combine(make_msg, data_type);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }

        private void sv_off_Click(object sender, EventArgs e)
        {
            var comm = servo_off("0");
            send_function(comm);
        }

        public byte[] servo_off(string channel)
        {
            byte[] command = Encoding.UTF8.GetBytes("DB");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] data_type = Encoding.UTF8.GetBytes("0");
            byte[] make_msg = Combine(command, channel_ba);
            make_msg = Combine(make_msg, data_type);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }

        private void error_reset_Click(object sender, EventArgs e)
        {
            var comm = error_reset_func();
            send_function(comm);
        }

        public byte[] error_reset_func()
        {
            byte[] make_msg = Encoding.UTF8.GetBytes("CG");
            byte lrc = lrc_cal(make_msg);
            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };
            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);
            return make_msg;
        }


        private void origin_Click(object sender, EventArgs e)
        {
            var comm = move_zero("0");
            send_function(comm);
        }



        public byte[] move_zero(string channel) // 원점 이동 부분
        {
            byte[] command = Encoding.UTF8.GetBytes("BA");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] make_msg = Combine(command, channel_ba);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }


        private async void setup_point_Click(object sender, EventArgs e)
        {
            richTextBox1.Text = "";
            this.Invoke(new MethodInvoker(delegate ()
            {
                progressBar1.Value = 0;
            }));

            decimal speed_1 = spd_value.Value;
            string spd = speed_1.ToString(spd_str);
            var comm = speed("0", spd);
            send_function(comm);

            x_stage_point = Convert.ToDouble(str_x.Value);
            y_stage_point = Convert.ToDouble(str_y.Value);
            z_stage_point = Convert.ToDouble(str_z.Value);

            var comm_xy = posi_check_robot("0");
            send_position(comm_xy);

            double x_set = Convert.ToDouble(x_position);
            double y_set = Convert.ToDouble(y_position);

            var task_chk = Task.Run(() =>
            {
                while (true)
                {
                    var comm_posi = posi_check_robot("0");
                    z_position_compare(comm_posi);

                    z_complete = Convert.ToDecimal(z_position);

                    if (z_complete == 0) // y축 좌표 비교 이동 완료
                    {
                        //var shutter_all = Task.Factory.StartNew(shutter_all_oneshot);
                        //shutter_all.Wait();                                   

                        //var shutter_check = Task.Factory.StartNew(shutter_open_sc10);
                        //shutter_check.Wait();
                        //this.Invoke(new Action(setup_ok_print));

                        break;
                    }
                    else
                    {
                        //this.Invoke(new Action(setup_cancel_print));

                        var comm_down_z = move_axis_all_z("0", x_set, y_set, 0);
                        send_function(comm_down_z);

                        //send_function(comm_z_up);
                    }
                }
            });
            task_chk.Wait();


            

            Thread.Sleep(300);


            //send_function(comm_1);

            x_start = x_stage_point;
            y_start = y_stage_point;
            z_start = z_stage_point;

            double x_setup_db = CustomRound(RoundType.Truncate, x_stage_point, 2);
            double y_setup_db = CustomRound(RoundType.Truncate, y_stage_point, 2);
            double z_setup_db = CustomRound(RoundType.Truncate, z_stage_point, 2);

            var task_setup = Task.Run(() =>
            {
                while (true)
                {
                    var comm_posi = posi_check_robot("0");
                    send_position(comm_posi);

                    x_complete = Convert.ToDecimal(x_position);
                    y_complete = Convert.ToDecimal(y_position);
                    //z_complete = Convert.ToDecimal(z_position);

                    if (x_complete == Convert.ToDecimal(x_setup_db) && y_complete == Convert.ToDecimal(y_setup_db))
                        //&& z_complete == Convert.ToDecimal(z_setup_db)) // y축 좌표 비교 이동 완료
                    {

                        var comm_down_z = move_axis_all_z("0", x_stage_point, y_stage_point, z_stage_point);
                        send_function(comm_down_z);

                        this.Invoke(new Action(setup_ok_print));
                        MessageBox.Show("Start Point OK !!");
                        break;
                    }
                    else
                    {
                        var comm_1 = move_axis_all("0", x_stage_point, y_stage_point, 0);
                        send_function(comm_1);

                    }
                }
            });

            //setup_point.Enabled = false;
            await task_setup;//.Wait();

            /*
            var task_setup_z = Task.Run(() =>
            {
                if (y_complete >= 43)
                {
                    var comm_down_z = move_axis_all_z("0", x_stage_point, y_stage_point, z_stage_point);
                    send_function(comm_down_z);


                }
            });
            //setup_point.Enabled = false;
            task_setup_z.Wait();
            */

        }

        public byte[] move_axis_all(string channel, double x_axis_point, double y_axis_point, double z_axis_point)
        {
            byte[] command = Encoding.UTF8.GetBytes("BC");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] motion_type = Encoding.UTF8.GetBytes("0");
            byte[] xy_type = Encoding.UTF8.GetBytes("1");

            decimal null_byte = 0;
            string null_st_fn = null_byte.ToString(ltn_stra2);

            decimal x_st = Convert.ToDecimal(x_axis_point);
            string x_st_fn = x_st.ToString(ltn_stra2);

            decimal y_st = Convert.ToDecimal(y_axis_point);
            string y_st_fn = y_st.ToString(ltn_stra2);

            decimal z_st = Convert.ToDecimal(z_axis_point);
            string z_st_fn = z_st.ToString(ltn_stra2);


            byte[] xy_location1 = Encoding.UTF8.GetBytes(x_st_fn);
            byte[] xy_location_null = Encoding.UTF8.GetBytes(null_st_fn);

            byte[] xy_location2 = Encoding.UTF8.GetBytes(y_st_fn);
            byte[] xy_location_null2 = Encoding.UTF8.GetBytes(y_st_fn);

            byte[] xy_location3 = Encoding.UTF8.GetBytes(z_st_fn);
            byte[] xy_location_null3 = Encoding.UTF8.GetBytes(z_st_fn);

            byte[] xy_location4 = Encoding.UTF8.GetBytes(z_st_fn);
            byte[] xy_location_null4 = Encoding.UTF8.GetBytes(z_st_fn);

            byte[] xy_location_final = Combine(xy_location1, xy_location2);
            //byte[] xy_location_final1 = Combine(xy_location2, xy_location3);
            byte[] xy_location_final2 = Combine(xy_location3, xy_location4);

            //byte[] xy_location_final = Combine(xy_location1, xy_location_null);
            //xy_location_final = Combine(xy_location_final, xy_location2);
            //xy_location_final = Combine(xy_location_final, xy_location_null2);

            byte[] make_msg = Combine(command, channel_ba);
            make_msg = Combine(make_msg, motion_type);
            make_msg = Combine(make_msg, xy_type);

            make_msg = Combine(make_msg, xy_location_final);
            //make_msg = Combine(make_msg, xy_location_final1);
            make_msg = Combine(make_msg, xy_location_final2);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }

        private void start_x_move_Click(object sender, EventArgs e)
        {
            decimal speed_1 = spd_value.Value;
            string spd = speed_1.ToString(spd_str);
            var comm = speed("0", spd);
            send_function(comm);

            Thread.Sleep(1000);

            double x_stage_point = Convert.ToDouble(str_x.Value);

            var comm_1 = move_axis_x("0", x_stage_point);
            send_function(comm_1);
        }

        private void start_y_move_Click(object sender, EventArgs e)
        {
            decimal speed_1 = spd_value.Value;
            string spd = speed_1.ToString(spd_str);
            var comm = speed("0", spd);
            send_function(comm);

            Thread.Sleep(1000);

            double y_stage_point = Convert.ToDouble(str_y.Value);

            var comm_1 = move_axis_y("0", y_stage_point);
            send_function(comm_1);
        }

        private void start_z_move_Click(object sender, EventArgs e)
        {
            decimal speed_1 = spd_value.Value;
            string spd = speed_1.ToString(spd_str);
            var comm = speed("0", spd);
            send_function(comm);

            Thread.Sleep(1000);

            double z_stage_point = Convert.ToDouble(str_z.Value);

            var comm_1 = move_axis_z("0", z_stage_point);
            send_function(comm_1);
        }

        public byte[] move_axis_x(string channel, double x_axis_point)
        {
            byte[] command = Encoding.UTF8.GetBytes("BC");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] motion_type = Encoding.UTF8.GetBytes("0");
            byte[] xy_type = Encoding.UTF8.GetBytes("1");

            decimal null_byte = 0;
            string null_st_fn = null_byte.ToString(ltn_stra2);

            decimal x_st = Convert.ToDecimal(x_axis_point);
            string x_st_fn = x_st.ToString(ltn_stra2);

            byte[] location_x = Encoding.UTF8.GetBytes(x_st_fn);
            byte[] location_null_x = Encoding.UTF8.GetBytes(null_st_fn);

            //byte[] xy_location_final = Combine(xy_location1, xy_location_null);

            //byte[] xy_location_final1 = Combine(xy_location2, xy_location3);
            //byte[] xy_location_final2 = Combine(xy_location3, xy_location4);

            //byte[] xy_location_final = Combine(xy_location1, xy_location_null);
            //xy_location_final = Combine(xy_location_final, xy_location2);
            //xy_location_final = Combine(xy_location_final, xy_location_null2);

            byte[] make_msg = Combine(command, channel_ba);
            make_msg = Combine(make_msg, motion_type);
            make_msg = Combine(make_msg, xy_type);

            make_msg = Combine(make_msg, location_x);
            make_msg = Combine(make_msg, location_null_x);
            make_msg = Combine(make_msg, location_null_x);
            make_msg = Combine(make_msg, location_null_x);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }


        public byte[] move_axis_y(string channel, double y_axis_point)
        {
            byte[] command = Encoding.UTF8.GetBytes("BC");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] motion_type = Encoding.UTF8.GetBytes("0");
            byte[] xy_type = Encoding.UTF8.GetBytes("1");

            decimal null_byte = 0;
            string null_st_fn = null_byte.ToString(ltn_stra2);
            ;
            decimal y_st = Convert.ToDecimal(y_axis_point);
            string y_st_fn = y_st.ToString(ltn_stra2);

            byte[] location_y = Encoding.UTF8.GetBytes(y_st_fn);
            byte[] location_null_y = Encoding.UTF8.GetBytes(null_st_fn);

            //byte[] location_final = Combine(xy_location1, xy_location2);
            //byte[] xy_location_final1 = Combine(xy_location2, xy_location3);
            //byte[] location_final2 = Combine(xy_location3, xy_location4);

            //byte[] xy_location_final = Combine(xy_location1, xy_location_null);
            //xy_location_final = Combine(xy_location_final, xy_location2);
            //xy_location_final = Combine(xy_location_final, xy_location_null2);

            byte[] make_msg = Combine(command, channel_ba);
            make_msg = Combine(make_msg, motion_type);
            make_msg = Combine(make_msg, xy_type);

            make_msg = Combine(make_msg, location_null_y);
            make_msg = Combine(make_msg, location_y);
            make_msg = Combine(make_msg, location_null_y);
            make_msg = Combine(make_msg, location_null_y);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }
        public byte[] move_axis_z(string channel, double z_axis_point)
        {
            byte[] command = Encoding.UTF8.GetBytes("BC");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] motion_type = Encoding.UTF8.GetBytes("0");
            byte[] xy_type = Encoding.UTF8.GetBytes("1");

            decimal null_byte = 0;
            string null_st_fn = null_byte.ToString(ltn_stra2);

            //decimal x_st = Convert.ToDecimal(x_axis_point);
            //string x_st_fn = x_st.ToString(ltn_stra2);

            //decimal y_st = Convert.ToDecimal(y_axis_point);
            //string y_st_fn = y_st.ToString(ltn_stra2);

            decimal z_st = Convert.ToDecimal(z_axis_point);
            string z_st_fn = z_st.ToString(ltn_stra2);


            byte[] location_z1 = Encoding.UTF8.GetBytes(z_st_fn);
            byte[] location_null_z1 = Encoding.UTF8.GetBytes(null_st_fn);

            //byte[] location_z2 = Encoding.UTF8.GetBytes(z_st_fn);
            //byte[] location_null_z2 = Encoding.UTF8.GetBytes(null_st_fn);

            //byte[] xy_location_final = Combine(xy_location1, xy_location2);
            //byte[] xy_location_final1 = Combine(xy_location2, xy_location3);
            //byte[] xy_location_final2 = Combine(xy_location3, xy_location4);

            //byte[] xy_location_final = Combine(xy_location1, xy_location_null);
            //xy_location_final = Combine(xy_location_final, xy_location2);
            //xy_location_final = Combine(xy_location_final, xy_location_null2);

            byte[] make_msg = Combine(command, channel_ba);
            make_msg = Combine(make_msg, motion_type);
            make_msg = Combine(make_msg, xy_type);

            make_msg = Combine(make_msg, location_null_z1);
            make_msg = Combine(make_msg, location_null_z1);
            make_msg = Combine(make_msg, location_z1);
            make_msg = Combine(make_msg, location_z1);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }


        public double x_right_move(double previous_x)
        {
            double x_pitch_size = Convert.ToDouble(pitch_x.Value);
            double x_move_right = Convert.ToDouble(previous_x) + x_pitch_size;
            return x_move_right;
        }

        public double x_left_move(double previous_x)
        {
            double x_pitch_size = Convert.ToDouble(pitch_x.Value);
            double x_move_left = Convert.ToDouble(previous_x) - x_pitch_size;
            return x_move_left;
        }


        public double y_down_move(double previous_y)
        {
            double y_pitch_size = Convert.ToDouble(pitch_y.Value);
            double y_move_down = Convert.ToDouble(previous_y) - y_pitch_size;
            return y_move_down;
        }

        public double y_up_move(double previous_y)
        {
            double y_pitch_size = Convert.ToDouble(pitch_y.Value);
            double y_move_down = Convert.ToDouble(previous_y) + y_pitch_size;
            return y_move_down;
        }

        public double z_up_move(double previous_z)
        {

            double z_pitch_size = Convert.ToDouble(pitch_z.Value);
            double z_move_up = Convert.ToDouble(previous_z) + z_pitch_size;
            w_stage_point = z_move_up;

            return z_move_up;
        }



        public double z_down_move(double previous_z)
        {
            double z_pitch_size = Convert.ToDouble(pitch_z.Value);
            double z_move_down = Convert.ToDouble(previous_z) - z_pitch_size;
            w_stage_point = z_move_down;

            return z_move_down;
        }

        private void error_check_Click(object sender, EventArgs e)
        {
            textBox2.Text = "";

            Thread.Sleep(300);

            var comm_1 = error_chk();
            send_error(comm_1);

            this.Invoke(new Action(error_print));

        }

        public void error_print()
        {
            textBox2.Text = error_result_st;
        }


        public byte[] error_chk() //error chk
        {
            byte[] make_msg = Encoding.UTF8.GetBytes("KD");

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }

        private void shutter_connect_Click(object sender, EventArgs e)
        {
            //serialNo1 = "68250369";
            //serialNo2 = "68250413";
            //serialNo3 = "68250412";

            try
            {
                // Tell the device manager to get the list of all devices connected to the computer
                DeviceManagerCLI.BuildDeviceList();
            }
            catch (Exception ex)
            {
                // An error occurred - see ex for details
                Console.WriteLine("Exception raised by BuildDeviceList {0}", ex);
                //Console.ReadKey();
                return;
            }

            // Get available KCube Solenoid and check our serial number is correct - by using the device prefix
            // (i.e. for serial number 68000123, the device prefix is 68)
            List<string> serialNumbers = DeviceManagerCLI.GetDeviceList(KCubeSolenoid.DevicePrefix);

            if (!serialNumbers.Contains(serialNo1))
            {
                // The requested serial number is not a KSC or is not connected
                Console.WriteLine("{0} is not a valid serial number", serialNo1);
                //Console.ReadKey();
                return;
            }

            // Create the device
            device1 = KCubeSolenoid.CreateKCubeSolenoid(serialNo1);
            device2 = KCubeSolenoid.CreateKCubeSolenoid(serialNo2);
            device3 = KCubeSolenoid.CreateKCubeSolenoid(serialNo3);

            if (device1 == null)
            {
                // An error occured
                Console.WriteLine("{0} is not a KCubeSolenoid", serialNo1);

                //Console.ReadKey();
                return;
            }

            // Open a connection to the device.
            try
            {
                Console.WriteLine("Opening device {0}", serialNo1);

                device1.Connect(serialNo1);
                device2.Connect(serialNo2);
                device3.Connect(serialNo3);

            }
            catch (Exception)
            {
                // Connection failed
                Console.WriteLine("Failed to open device {0}", serialNo1);
                // Console.ReadKey();
                return;
            }

            // Wait for the device settings to initialize - timeout 5000ms
            if (!device1.IsSettingsInitialized())
            {
                try
                {
                    device1.WaitForSettingsInitialized(5000);
                }
                catch (Exception)
                {
                    Console.WriteLine("Settings failed to initialize");
                }
            }

            if (!device2.IsSettingsInitialized())
            {
                try
                {
                    device2.WaitForSettingsInitialized(5000);
                }
                catch (Exception)
                {
                    Console.WriteLine("Settings failed to initialize");
                }
            }

            if (!device3.IsSettingsInitialized())
            {
                try
                {
                    device3.WaitForSettingsInitialized(5000);
                }
                catch (Exception)
                {
                    Console.WriteLine("Settings failed to initialize");
                }
            }
            // Display info about device
            DeviceInfo deviceInfo1 = device1.GetDeviceInfo();
            DeviceInfo deviceInfo2 = device2.GetDeviceInfo();
            DeviceInfo deviceInfo3 = device3.GetDeviceInfo();

            Console.WriteLine("Device {0} = {1}", deviceInfo1.SerialNumber, deviceInfo1.Name);
            Console.WriteLine("Device {0} = {1}", deviceInfo2.SerialNumber, deviceInfo2.Name);
            Console.WriteLine("Device {0} = {1}", deviceInfo3.SerialNumber, deviceInfo3.Name);

            // Start the device polling
            // The polling loop requests regular status requests to the motor to ensure the program keeps track of the device. 
            device1.StartPolling(250);
            device2.StartPolling(250);
            device3.StartPolling(250);

            // Needs a delay so that the current enabled state can be obtained
            Thread.Sleep(500);
            // Enable the channel otherwise any move is ignored 
            device1.EnableDevice();
            device2.EnableDevice();
            device3.EnableDevice();
            // Needs a delay to give time for the device to be enabled
            Thread.Sleep(500);

            // get Device Configuration
            SolenoidConfiguration solenoidConfiguration1 = device1.GetSolenoidConfiguration(serialNo1);
            ThorlabsKCubeSolenoidSettings currentDeviceSettings1 = ThorlabsKCubeSolenoidSettings.GetSettings(solenoidConfiguration1);

            SolenoidConfiguration solenoidConfiguration2 = device2.GetSolenoidConfiguration(serialNo2);
            ThorlabsKCubeSolenoidSettings currentDeviceSettings2 = ThorlabsKCubeSolenoidSettings.GetSettings(solenoidConfiguration2);

            SolenoidConfiguration solenoidConfiguration3 = device3.GetSolenoidConfiguration(serialNo3);
            ThorlabsKCubeSolenoidSettings currentDeviceSettings3 = ThorlabsKCubeSolenoidSettings.GetSettings(solenoidConfiguration3);




            bool shutter1_connect = DeviceManagerCLI.IsDeviceConnected(serialNo1);
            bool shutter2_connect = DeviceManagerCLI.IsDeviceConnected(serialNo2);
            bool shutter3_connect = DeviceManagerCLI.IsDeviceConnected(serialNo3);

            if (shutter1_connect && shutter2_connect && shutter3_connect == true)
            {
                textBox4.Text = ("Shutter Connected");
                shutter_connect.Enabled = false;
            }

        }

        private void shutter1_oneshot()
        {
            device1.SetOperatingState(SolenoidStatus.OperatingStates.Active);

            //Thread.Sleep(settings.OpenTime / 2);
            //Thread.Sleep(200);

            while (true)
            {
                signal1 = device1.GetStatusBits();

                if (signal1 == 8192)
                    break;
                else
                {
                    Thread.Sleep(100);
                }
            }
        }

        private void shutter2_oneshot()
        {
            device2.SetOperatingState(SolenoidStatus.OperatingStates.Active);

            //Thread.Sleep(settings.OpenTime / 2);
            //Thread.Sleep(200);

            while (true)
            {
                signal2 = device2.GetStatusBits();

                if (signal2 == 8192)
                    break;
                else
                {
                    Thread.Sleep(100);
                }
            }
        }

        private void shutter3_oneshot()
        {
            device3.SetOperatingState(SolenoidStatus.OperatingStates.Active);
            //Thread.Sleep(200);

            //Thread.Sleep(settings.OpenTime / 2);

            while (true)
            {
                signal3 = device3.GetStatusBits();

                if (signal3 == 8192)
                    break;
                else
                {
                    Thread.Sleep(100);
                }
            }
        }
        private void shutter_all_oneshot()
        {
            device1.SetOperatingState(SolenoidStatus.OperatingStates.Active);
            device2.SetOperatingState(SolenoidStatus.OperatingStates.Active);
            device3.SetOperatingState(SolenoidStatus.OperatingStates.Active);

            Thread.Sleep(settings.OpenTime / 2);

            while (true)
            {
                signal1 = device1.GetStatusBits();
                signal2 = device2.GetStatusBits();
                signal3 = device3.GetStatusBits();

                if (signal1 == 8192 && signal2 == 8192 && signal3 == 8192)
                    break;
            }
        }

        private void shutter_open_sc10()
        {

            //open = 49, close = 48

            //while문 check
            _serialPort.WriteLine("ens");

            //_serialPort.DiscardOutBuffer();
            _serialPort.DiscardInBuffer();

            while (true)
            {
                _serialPort.WriteLine("ens?");
                _serialPort.DiscardOutBuffer();

                Thread.Sleep(100);

                string closed1 = _serialPort.ReadExisting();
                bool closed = closed1.Contains("0");

                if (closed)
                {
                    //textBox2.Text = "done";
                    break;
                }
                closed1 = "";
            }

            /*
            device1.SetOperatingState(SolenoidStatus.OperatingStates.Active);
            device2.SetOperatingState(SolenoidStatus.OperatingStates.Active);
            device3.SetOperatingState(SolenoidStatus.OperatingStates.Active);

            Thread.Sleep(settings.OpenTime / 2);

            while (true)
            {
                signal1 = device1.GetStatusBits();
                signal2 = device2.GetStatusBits();
                signal3 = device3.GetStatusBits();

                if (signal1 == 8192 && signal2 == 8192 && signal3 == 8192)
                    break;
            }
            */

        }

        private void open_1_Click(object sender, EventArgs e)
        {
            device1.SetOperatingMode(SolenoidStatus.OperatingModes.Manual);
            device1.SetOperatingState(SolenoidStatus.OperatingStates.Active);
        }

        private void close_1_Click(object sender, EventArgs e)
        {
            device1.SetOperatingMode(SolenoidStatus.OperatingModes.Manual);
            device1.SetOperatingState(SolenoidStatus.OperatingStates.Inactive);
        }

        private void open_2_Click(object sender, EventArgs e)
        {
            device2.SetOperatingMode(SolenoidStatus.OperatingModes.Manual);
            device2.SetOperatingState(SolenoidStatus.OperatingStates.Active);
        }

        private void close_2_Click(object sender, EventArgs e)
        {
            device2.SetOperatingMode(SolenoidStatus.OperatingModes.Manual);
            device2.SetOperatingState(SolenoidStatus.OperatingStates.Inactive);
        }

        private void open_3_Click(object sender, EventArgs e)
        {
            device3.SetOperatingMode(SolenoidStatus.OperatingModes.Manual);
            device3.SetOperatingState(SolenoidStatus.OperatingStates.Active);
        }

        private void close_3_Click(object sender, EventArgs e)
        {
            device3.SetOperatingMode(SolenoidStatus.OperatingModes.Manual);
            device3.SetOperatingState(SolenoidStatus.OperatingStates.Inactive);
        }

        private void all_open_Click(object sender, EventArgs e)
        {
            device1.SetOperatingMode(SolenoidStatus.OperatingModes.Manual);
            device2.SetOperatingMode(SolenoidStatus.OperatingModes.Manual);
            device3.SetOperatingMode(SolenoidStatus.OperatingModes.Manual);

            device1.SetOperatingState(SolenoidStatus.OperatingStates.Active);
            device2.SetOperatingState(SolenoidStatus.OperatingStates.Active);
            device3.SetOperatingState(SolenoidStatus.OperatingStates.Active);
        }



        private void all_close_Click(object sender, EventArgs e)
        {
            device1.SetOperatingMode(SolenoidStatus.OperatingModes.Manual);
            device2.SetOperatingMode(SolenoidStatus.OperatingModes.Manual);
            device3.SetOperatingMode(SolenoidStatus.OperatingModes.Manual);

            device1.SetOperatingState(SolenoidStatus.OperatingStates.Inactive);
            device2.SetOperatingState(SolenoidStatus.OperatingStates.Inactive);
            device3.SetOperatingState(SolenoidStatus.OperatingStates.Inactive);
        }

        private async void one_shot_Click(object sender, EventArgs e)
        {
            settings.OpenTime = (Convert.ToInt16(exposure_time.Value) * 1000);
            settings.ClosedTime = 1000;

            device1.SetCycleParams(settings);
            device2.SetCycleParams(settings);
            device3.SetCycleParams(settings);

            device1.SetOperatingMode(SolenoidStatus.OperatingModes.SingleToggle);
            device2.SetOperatingMode(SolenoidStatus.OperatingModes.SingleToggle);
            device3.SetOperatingMode(SolenoidStatus.OperatingModes.SingleToggle);

            var shutter1 = Task.Factory.StartNew(shutter1_oneshot);

            shutter1.Wait();
            await shutter1;

            var shutter2 = Task.Factory.StartNew(shutter2_oneshot);

            await shutter2;
            shutter2.Wait();

            var shutter3 = Task.Factory.StartNew(shutter3_oneshot);
            //await shutter3;
            shutter3.Wait();

        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }



        private void xy_position_compare(byte[] msg) //좌표 수신
        {
            byte[] bytes = new byte[50];

            byte[] ack = new byte[] { ACK };
            byte[] header = new byte[] { stx, dummy };

            msg = Combine(header, msg);

            if (sock.Available > 0) // here we clean up the current queue
            {
                sock.Receive(bytes);
                //sock1.Receive(bytes1);
            }

            sock.Send(msg);

            while (sock.Available == 0) // wait for the controller response
            {
                Thread.Sleep(10);
            }

            sock.Receive(bytes); // after receiving the data, we should check the LRC if possible

            if (bytes.Contains<byte>(nak) || bytes.Contains<byte>(rst) == true)
            {
                sock.Send(msg);
            }
            else
            {
                sock.Send(ack);
            }

            status = Encoding.UTF8.GetString(bytes);

            status.CopyTo(3, x_pos, 0, 7);
            status.CopyTo(13, y_pos, 0, 7);

            //int val = Convert.ToInt32(x_pos[8]);
            //x_pos[8] = Convert.ToChar(48);//Convert.ToChar(val - 1);

            //x_position = new string(x_pos);//비교를 위해 초기값 저장
            //x_position = Math.Abs(x_position);

            //x_position = x_position.Trim();

            x_position = new string(x_pos);//비교를 위해 초기값 저장
            x_position = x_position.Trim();

            y_position = new string(y_pos);//비교를 위해 초기값 저장
            y_position = y_position.Trim();

            msg.Initialize();
            bytes.Initialize();
        }

        private void z_position_compare(byte[] msg) //좌표 수신
        {
            byte[] bytes = new byte[50];

            byte[] ack = new byte[] { ACK };
            byte[] header = new byte[] { stx, dummy };

            msg = Combine(header, msg);

            if (sock.Available > 0) // here we clean up the current queue
            {
                sock.Receive(bytes);
                //sock1.Receive(bytes1);
            }

            sock.Send(msg);

            while (sock.Available == 0) // wait for the controller response
            {
                Thread.Sleep(10);
            }

            sock.Receive(bytes); // after receiving the data, we should check the LRC if possible

            if (bytes.Contains<byte>(nak) || bytes.Contains<byte>(rst) == true)
            {
                sock.Send(msg);
            }
            else
            {
                sock.Send(ack);
            }

            status = Encoding.UTF8.GetString(bytes);


            //status.CopyTo(3, x_pos, 0, 10);
            //status.CopyTo(13, y_pos, 0, 10);
            status.CopyTo(23, z_pos, 0, 7);
            status.CopyTo(33, w_pos, 0, 7);

            //int val = Convert.ToInt32(x_pos[8]);
            //x_pos[8] = Convert.ToChar(48);//Convert.ToChar(val - 1);

            //x_position = new string(x_pos);//비교를 위해 초기값 저장
            //x_position = Math.Abs(x_position);

            //x_position = x_position.Trim();

            z_position = new string(z_pos);//비교를 위해 초기값 저장
            z_position = z_position.Trim();

            w_position = new string(w_pos);//비교를 위해 초기값 저장
            w_position = w_position.Trim();

            msg.Initialize();
            bytes.Initialize();
        }

        private async void onestep_Click(object sender, EventArgs e)
        {
            open_time_sc10 = (Convert.ToInt16(exposure_time.Value) * 1000);
            string open_time_val = "open=" + Convert.ToString(open_time_sc10);
            _serialPort.WriteLine("mode=3");
            _serialPort.WriteLine(open_time_val);
            _serialPort.WriteLine("shut=1");

            double pitch_size_x = Convert.ToDouble(pitch_x.Value);
            double pitch_size_y = Convert.ToDouble(pitch_y.Value);
            double pitch_size_z = Convert.ToDouble(pitch_z.Value);

            double origin_x = Convert.ToDouble(str_x.Value);
            double origin_y = Convert.ToDouble(str_y.Value);
            double origin_z = Convert.ToDouble(str_z.Value);

            //double y_limt = y_start + (pitch_size_y * (y_count_1 - 1));
            //double x_limt = x_start + (pitch_size_x * (x_count_1 - 1));
            double z_limt = z_start + pitch_size_z;


            decimal speed_1 = spd_value.Value;
            string spd_1 = speed_1.ToString(spd_str);
            var comm_1 = speed("0", spd_1);
            send_function(comm_1);

            if (z_limt > 45)//z 센서 거리 초과
            {
                //var comm = move_axis("0", x_stage_point, y_stage_point);
                //send2controller(comm);
                richTextBox1.Text = ("OVER Z-AXIS LIMIT");
            }

            if (richTextBox1.Text == "setup")
            {
                //if (i % 2 == 0) //y카운트 짝수일 때
                //{
                z_stage_point = z_up_move(z_stage_point);
                var comm_z_up = move_axis_all_z("0", x_stage_point, y_stage_point, z_stage_point);
                send_function(comm_z_up);

                z_up_db = CustomRound(RoundType.Truncate, z_stage_point, 2);

                Thread.Sleep(500);

                var task_even_1 = Task.Run(() =>
                {
                    while (true)
                    {
                        var comm_posi = posi_check_robot("0");
                        z_position_compare(comm_posi);

                        z_complete = Convert.ToDecimal(z_position);

                        if (z_complete == Convert.ToDecimal(z_up_db)) // y축 좌표 비교 이동 완료
                        {
                            //var shutter_all = Task.Factory.StartNew(shutter_all_oneshot);
                            //shutter_all.Wait();                                   

                            var shutter_check = Task.Factory.StartNew(shutter_open_sc10);
                            shutter_check.Wait();

                            break;
                        }
                        else
                        {
                            send_function(comm_z_up);
                        }
                    }
                });
                //task_even_1.Wait();
                await task_even_1;

                z_stage_point = z_down_move(z_stage_point);
                var comm_down_z = move_axis_all_z("0", x_stage_point, y_stage_point, z_stage_point);
                send_function(comm_down_z);

                z_down_db = CustomRound(RoundType.Truncate, z_stage_point, 2);

                Thread.Sleep(500);

                var task_even_2 = Task.Run(() =>
                {
                    while (true)
                    {
                        var comm_posi = posi_check_robot("0");
                        z_position_compare(comm_posi);

                        z_complete = Convert.ToDecimal(z_position);

                        if (z_complete == Convert.ToDecimal(z_down_db)) // y축 좌표 비교 이동 완료
                        {
                            break;
                        }
                        else
                        {
                            send_function(comm_down_z);
                        }
                    }
                });
                //task_even_2.Wait();
                await task_even_2;
                //}
            }
        }



        private async void all_step_Click(object sender, EventArgs e)
        {
            open_time_sc10 = (Convert.ToInt16(exposure_time.Value) * 1000);
            string open_time_val = "open=" + Convert.ToString(open_time_sc10);
            _serialPort.WriteLine("mode=3");
            _serialPort.WriteLine(open_time_val);
            _serialPort.WriteLine("shut=1");

            int x_count_1 = Convert.ToInt32(x_array.Value);
            int y_count_1 = Convert.ToInt32(y_array.Value);

            double pitch_size_x = Convert.ToDouble(pitch_x.Value);
            double pitch_size_y = Convert.ToDouble(pitch_y.Value);
            double pitch_size_z = Convert.ToDouble(pitch_z.Value);

            double origin_x = Convert.ToDouble(str_x.Value);
            double origin_y = Convert.ToDouble(str_y.Value);
            double origin_z = Convert.ToDouble(str_z.Value);

            //double y_limt = y_start + (pitch_size_y * (y_count_1 - 1));
            double y_limt = y_start - (pitch_size_y * (y_count_1 - 1));
            double x_limt = x_start + (pitch_size_x * (x_count_1 - 1));
            double z_limt = z_start + pitch_size_z;

            //string spd = spd_value.Value.ToString();

            //double x_stage_point = Convert.ToDouble(str_x.Value);
            //double y_stage_point = Convert.ToDouble(str_y.Value);
            //double z_stage_point = Convert.ToDouble(str_z.Value);

            int end_count = y_count_1;

            this.Invoke(new MethodInvoker(delegate ()
            {
                progressBar1.Maximum = end_count + 1;
            }));

            //x_point_db = 0;
            //y_point_db = 0;

            /*
            settings.OpenTime = (Convert.ToInt16(exposure_time.Value) * 1000);
            settings.ClosedTime = 0;
            settings.NumberOfCycles = 0;

            device1.SetCycleParams(settings);
            device2.SetCycleParams(settings);
            device3.SetCycleParams(settings);

            device1.SetOperatingMode(SolenoidStatus.OperatingModes.SingleToggle);
            device2.SetOperatingMode(SolenoidStatus.OperatingModes.SingleToggle);
            device3.SetOperatingMode(SolenoidStatus.OperatingModes.SingleToggle);
            */
            //ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
            //셔터 싱글토글

            //var shutter1 = Task.Factory.StartNew(shutter1_oneshot);
            //await shutter1;

            //var shutter2 = Task.Factory.StartNew(shutter2_oneshot);
            //await shutter2;

            //var shutter3 = Task.Factory.StartNew(shutter3_oneshot);
            //await shutter3;

            //var shutter_all = Task.Factory.StartNew(shutter_all_oneshot);
            //await shutter_all;

            decimal speed_1 = spd_value.Value;
            string spd_1 = speed_1.ToString(spd_str);
            var comm_1 = speed("0", spd_1);
            send_function(comm_1);

            //if (y_limt > 455)//y 센서 거리 초과
            if (y_limt < 0)//y 센서 거리 초과
            {
                //var comm = move_axis("0", x_stage_point, y_stage_point);
                //send2controller(comm);
                richTextBox1.Text = ("OVER Y-AXIS LIMIT");
            }

            else if (x_limt > 690)//x 센서 거리 초과
            {
                //var comm = move_axis("0", x_stage_point, y_stage_point);
                //send2controller(comm);
                richTextBox1.Text = ("OVER X-AXIS LIMIT");
            }

            else if (z_limt > 45)//z 센서 거리 초과
            {
                //var comm = move_axis("0", x_stage_point, y_stage_point);
                //send2controller(comm);
                richTextBox1.Text = ("OVER Z-AXIS LIMIT");
            }

            //x축 6번(7), y축 3번 7*3 = 21번 기록

            if (richTextBox1.Text == "setup")
            {
                for (int i = 0; i < y_count_1; i++) //y가 1씩 증가할 때
                {
                    this.Invoke(new MethodInvoker(delegate ()
                    {
                        progressBar1.Value = i + 1;
                    }));

                    if (i % 2 == 0) //y카운트 짝수일 때
                    {
                        z_stage_point = z_up_move(z_stage_point);
                        var comm_z_up = move_axis_all_z("0", x_stage_point, y_stage_point, z_stage_point);
                        send_function(comm_z_up);

                        z_up_db = CustomRound(RoundType.Truncate, z_stage_point, 2);

                        Thread.Sleep(500);

                        var task_even_1 = Task.Run(() =>
                        {
                            while (true)
                            {
                                var comm_posi = posi_check_robot("0");
                                z_position_compare(comm_posi);

                                z_complete = Convert.ToDecimal(z_position);

                                if (z_complete == Convert.ToDecimal(z_up_db)) // y축 좌표 비교 이동 완료
                                {
                                    //var shutter_all = Task.Factory.StartNew(shutter_all_oneshot);
                                    //shutter_all.Wait();                                   

                                    this.Invoke(new MethodInvoker(delegate ()
                                    {
                                        //progressBar1.Value = i + 1;
                                        richTextBox1.AppendText("\r\n");
                                        richTextBox1.AppendText(x_stage_point.ToString() + "," + y_stage_point.ToString());
                                    }));


                                    var shutter_check = Task.Factory.StartNew(shutter_open_sc10);
                                    shutter_check.Wait();


                                 


                                    break;
                                }
                                else
                                {
                                    send_function(comm_z_up);
                                }
                            }
                        });
                        //task_even_1.Wait();
                        await task_even_1;

                        z_stage_point = z_down_move(z_stage_point);
                        var comm_down_z = move_axis_all_z("0", x_stage_point, y_stage_point, z_stage_point);
                        send_function(comm_down_z);

                        z_down_db = CustomRound(RoundType.Truncate, z_stage_point, 2);

                        Thread.Sleep(500);

                        var task_even_2 = Task.Run(() =>
                        {
                            while (true)
                            {
                                var comm_posi = posi_check_robot("0");
                                z_position_compare(comm_posi);

                                z_complete = Convert.ToDecimal(z_position);

                                if (z_complete == Convert.ToDecimal(z_down_db)) // y축 좌표 비교 이동 완료
                                {
                                    break;
                                }
                                else
                                {
                                    send_function(comm_down_z);
                                }
                            }
                        });
                        //task_even_2.Wait();
                        await task_even_2;

                        for (int m = 1; m < x_count_1; m++)//y카운트 짝수일 때
                        {
                            x_stage_point = x_right_move(x_stage_point);
                            var comm_even_x = move_axis_all_x("0", x_stage_point, y_stage_point, z_start);
                            send_function(comm_even_x);

                            double x_point_db = CustomRound(RoundType.Truncate, x_stage_point, 2);

                            Thread.Sleep(500);

                            var task_even_4 = Task.Run(() =>
                            {
                                while (true)
                                {
                                    var comm_posi_2 = posi_check_robot("0");
                                    xy_position_compare(comm_posi_2);

                                //MessageBox.Show("x");
                                    x_complete = Convert.ToDecimal(x_position);

                                    if (x_complete == Convert.ToDecimal(x_point_db)) // y축 좌표 비교 이동 완료
                                    {
                                        z_stage_point = z_up_move(z_stage_point);
                                        comm_z_up = move_axis_all_z("0", x_stage_point, y_stage_point, z_stage_point);
                                        send_function(comm_z_up);
                                    //x_point_db = 0;
                                        break;
                                    }
                                    else
                                    {
                                        send_function(comm_even_x);
                                    }
                                }
                            });
                            //task_even_4.Wait();
                            await task_even_4;

                            var task_even_5 = Task.Run(() =>
                            {
                                while (true)
                                {
                                    var comm_posi_z = posi_check_robot("0");
                                    z_position_compare(comm_posi_z);

                                    z_complete = Convert.ToDecimal(z_position);
                                    w_complete = Convert.ToDecimal(w_position);

                                    if (z_complete == Convert.ToDecimal(z_up_db))// && w_complete == Convert.ToDecimal(z_up_db)) // y축 좌표 비교 이동 완료
                                    {
                                        //var shutter_all = Task.Factory.StartNew(shutter_all_oneshot);
                                        //shutter_all.Wait();

                                        this.Invoke(new MethodInvoker(delegate ()
                                        {
                                            //progressBar1.Value = i + 1;
                                            richTextBox1.AppendText("\r\n");
                                            richTextBox1.AppendText(x_stage_point.ToString() + "," + y_stage_point.ToString());
                                        }));


                                        var shutter_check = Task.Factory.StartNew(shutter_open_sc10);
                                        shutter_check.Wait();




                                        break;
                                    }
                                    else
                                    {
                                        send_function(comm_z_up);
                                    }
                                }
                            });
                            //task_even_5.Wait();
                            await task_even_5;

                            z_stage_point = z_down_move(z_stage_point);
                            var comm_z_down = move_axis_all_z("0", x_stage_point, y_stage_point, z_stage_point);
                            send_function(comm_z_down);

                            Thread.Sleep(500);

                            var task_even_6 = Task.Run(() =>
                            {
                                while (true)
                                {
                                    var comm_posi_z1 = posi_check_robot("0");
                                    z_position_compare(comm_posi_z1);

                                    z_complete = Convert.ToDecimal(z_position);
                                    w_complete = Convert.ToDecimal(w_position);

                                    if (z_complete == Convert.ToDecimal(z_down_db))// && w_complete == Convert.ToDecimal(z_down_db)) // y축 좌표 비교 이동 완료
                                    {
                                        break;
                                    }
                                    else
                                    {
                                        send_function(comm_z_down);
                                    }
                                }
                            });
                            //task_even_6.Wait();
                            await task_even_6;

                            if (m == x_count_1 - 1)
                            {
                                if (i == end_count - 1)
                                {
                                    break;
                                }

                                
                                //y_stage_point = y_up_move(y_stage_point);
                                y_stage_point = y_down_move(y_stage_point);
                                
                                var comm_even_y_up = move_axis_all_y("0", x_stage_point, y_stage_point, z_start);
                                send_function(comm_even_y_up);

                                double y_point_db = CustomRound(RoundType.Truncate, y_stage_point, 2);

                                Thread.Sleep(500);

                                while (true) // y축 좌표 비교 이동 완료
                                {
                                    var comm_posi_3 = posi_check_robot("0");
                                    xy_position_compare(comm_posi_3);

                                    y_complete = Convert.ToDecimal(y_position);

                                    if (y_complete == Convert.ToDecimal(y_point_db)) // y축 좌표 비교 이동 완료
                                    {
                                        break;
                                    }
                                    else
                                    {
                                        send_function(comm_even_y_up);
                                    }
                                }
                            }
                        }

                        if (i == end_count - 1)
                        {
                            break;
                        }
                    }

                    else    //홀수
                    {
                        z_stage_point = z_up_move(z_stage_point);
                        var comm_z_up = move_axis_all_z("0", x_stage_point, y_stage_point, z_stage_point);
                        send_function(comm_z_up);

                        Thread.Sleep(500);

                        this.Invoke(new MethodInvoker(delegate ()
                        {
                            //progressBar1.Value = i + 1;
                            richTextBox1.AppendText("\r\n");
                            richTextBox1.AppendText(x_stage_point.ToString() + "," + y_stage_point.ToString());
                        }));

                        //z_up_db = CustomRound(RoundType.Truncate, z_stage_point, 2);

                        var task_odd = Task.Run(() =>
                        {
                            while (true)
                            {
                                var comm_posi = posi_check_robot("0");
                                z_position_compare(comm_posi);

                                z_complete = Convert.ToDecimal(z_position);

                                if (z_complete == Convert.ToDecimal(z_up_db)) // y축 좌표 비교 이동 완료
                                {
                                    //var shutter_all = Task.Factory.StartNew(shutter_all_oneshot);
                                    //shutter_all.Wait();

                                 

                                    var shutter_check = Task.Factory.StartNew(shutter_open_sc10);
                                    shutter_check.Wait();

                                    break;
                                }
                                else
                                {
                                    send_function(comm_z_up);
                                }
                            }
                        });
                        //task_odd.Wait();
                        await task_odd;//.Wait();

                        z_stage_point = z_down_move(z_stage_point);
                        var comm_down_z = move_axis_all_z("0", x_stage_point, y_stage_point, z_stage_point);
                        send_function(comm_down_z);

                        Thread.Sleep(500);

                        //double z_down_db = CustomRound(RoundType.Truncate, z_stage_point, 1);

                        var task_odd_1 = Task.Run(() =>
                        {
                            while (true)
                            {
                                var comm_posi = posi_check_robot("0");
                                z_position_compare(comm_posi);

                                z_complete = Convert.ToDecimal(z_position);

                                if (z_complete == Convert.ToDecimal(z_down_db)) // y축 좌표 비교 이동 완료
                                {

                                    break;
                                }
                                else
                                {
                                    send_function(comm_down_z);
                                }
                            }
                        });
                        //task_odd_1.Wait();
                        await task_odd_1;

                        for (int r = 1; r < x_count_1; r++) //홀수
                        {
                            x_stage_point = x_left_move(x_stage_point);
                            var comm_odd_x = move_axis_all_x("0", x_stage_point, y_stage_point, z_start);
                            send_function(comm_odd_x);

                            double x_point_db = CustomRound(RoundType.Truncate, x_stage_point, 2);

                            var task_odd_2 = Task.Run(() =>
                            {
                                Thread.Sleep(500);

                                while (true) // y축 좌표 비교 이동 완료
                                {
                                    var comm_posi_2 = posi_check_robot("0");
                                    xy_position_compare(comm_posi_2);

                                    x_complete = Convert.ToDecimal(x_position);

                                    if (x_complete == Convert.ToDecimal(x_point_db)) // y축 좌표 비교 이동 완료
                                    {
                                        z_stage_point = z_up_move(z_stage_point);
                                        comm_z_up = move_axis_all_z("0", x_stage_point, y_stage_point, z_stage_point);
                                        send_function(comm_z_up);

                                        this.Invoke(new MethodInvoker(delegate ()
                                        {
                                            //progressBar1.Value = i + 1;
                                            richTextBox1.AppendText("\r\n");
                                            richTextBox1.AppendText(x_stage_point.ToString() + "," + y_stage_point.ToString());
                                        }));

                                        break;
                                    }
                                    else
                                    {
                                        send_function(comm_odd_x);
                                    }
                                }
                            });
                            //task_odd_2.Wait();
                            await task_odd_2;

                            Thread.Sleep(500);

                            var task_odd_3 = Task.Run(() =>
                            {
                                while (true)
                                {
                                    var comm_posi_z = posi_check_robot("0");
                                    z_position_compare(comm_posi_z);

                                    z_complete = Convert.ToDecimal(z_position);
                                    w_complete = Convert.ToDecimal(w_position);

                                    if (z_complete == Convert.ToDecimal(z_up_db)) // y축 좌표 비교 이동 완료
                                    {
                                        //var shutter_all = Task.Factory.StartNew(shutter_all_oneshot);
                                        //shutter_all.Wait();

                                      

                                        var shutter_check = Task.Factory.StartNew(shutter_open_sc10);
                                        shutter_check.Wait();

                                        break;
                                    }
                                    else
                                    {
                                        send_function(comm_z_up);
                                    }
                                }
                            });
                            //task_odd_3.Wait();
                            await task_odd_3;

                            z_stage_point = z_down_move(z_stage_point);
                            var comm_z_down = move_axis_all_z("0", x_stage_point, y_stage_point, z_stage_point);
                            send_function(comm_z_down);

                            Thread.Sleep(500);

                            var task_odd_4 = Task.Run(() =>
                            {
                                while (true)
                                {
                                    var comm_posi_z1 = posi_check_robot("0");
                                    z_position_compare(comm_posi_z1);

                                    z_complete = Convert.ToDecimal(z_position);
                                    w_complete = Convert.ToDecimal(w_position);

                                    if (z_complete == Convert.ToDecimal(z_down_db)) // y축 좌표 비교 이동 완료
                                    {
                                        //Thread.Sleep(2000);
                                        //this.Invoke(new Action(zdown_ok_print));
                                        break;
                                    }
                                    else
                                    {
                                        send_function(comm_z_down);
                                    }
                                }
                            });
                            //task_odd_4.Wait();
                            await task_odd_4;

                            if (r == x_count_1 - 1)
                            {
                                if (i == end_count - 1)
                                {
                                    break;
                                }

                                //y_stage_point = y_up_move(y_stage_point);
                                y_stage_point = y_down_move(y_stage_point);
                                
                                var comm_odd_y_up = move_axis_all_y("0", x_stage_point, y_stage_point, z_start);
                                send_function(comm_odd_y_up);

                                Thread.Sleep(500);

                                double y_db = CustomRound(RoundType.Truncate, y_stage_point, 2);

                                var task_odd_end = Task.Run(() =>
                                {

                                    while (true) // y축 좌표 비교 이동 완료
                                    {
                                        var comm_posi_3 = posi_check_robot("0");
                                        xy_position_compare(comm_posi_3);

                                        y_complete = Convert.ToDecimal(y_position);

                                        if (y_complete == Convert.ToDecimal(y_db)) // y축 좌표 비교 이동 완료
                                        {
                                            break;
                                        }
                                        else
                                        {
                                            send_function(comm_odd_y_up);
                                        }
                                    }
                                });
                                //task_odd_end.Wait();
                                await task_odd_end;
                            }
                        }

                        if (i == end_count - 1)
                        {
                            break;
                        }

                    }
                }
                //var task_all_end = Task.Run(() =>
                //{
                this.Invoke(new MethodInvoker(delegate ()
                {
                    progressBar1.Value = end_count + 1;
                }));

                var comm_all_end = move_axis_all("0", 150, 290, z_start);
                send_function(comm_all_end);

                var task_setup = Task.Run(() =>
                {
                    while (true)
                    {
                        var comm_posi = posi_check_robot("0");
                        send_position(comm_posi);

                        x_complete = Convert.ToDecimal(x_position);
                        y_complete = Convert.ToDecimal(y_position);
                        z_complete = Convert.ToDecimal(z_position);

                        //if (x_complete == Convert.ToDecimal(x_start) && y_complete == Convert.ToDecimal(y_start)
                        //    && z_complete == Convert.ToDecimal(z_start)) // y축 좌표 비교 이동 완료
                        if (x_complete == 150 && y_complete == 290
                            && z_complete == Convert.ToDecimal(z_start)) // y축 좌표 비교 이동 완료
                        {
                            this.Invoke(new Action(finish_print));
                            break;
                        }
                        else
                        {
                            send_function(comm_all_end);
                        }
                    }
                });
                await task_setup;

                setup_point.Enabled = true;

                //});
                //await task_all_end;
                x_start = 0;
                y_start = 0;
                z_start = 0;

                x_complete = 0;
                y_complete = 0;
                z_complete = 0;

                MessageBox.Show("Finish Job");
            }
        }

        public void shutter_open_print()
        {
            richTextBox1.ResetText();
            richTextBox1.AppendText("shutter open");
        }
        public void shutter_close_print()
        {
            richTextBox1.ResetText();
            richTextBox1.AppendText("shutter close");

        }
        public void setup_ok_print()
        {
            richTextBox1.ResetText();
            richTextBox1.Text = "setup";
        }

        public void setup_cancel_print()
        {
            richTextBox1.ResetText();
            richTextBox1.Text = "z-axis down";
        }


        public void zdown_ok_print()
        {
            richTextBox1.ResetText();
            richTextBox1.AppendText("Z-Axis down ok");

        }
        public void finish_print()
        {
            richTextBox1.ResetText();
            richTextBox1.AppendText("job finish");
        }

        public void position_print()
        {
            //textBox3.ResetText();
            //textBox3.ResetText();
            //textBox3.AppendText();
        }


        public byte[] move_axis_all_x(string channel, double x_axis_point, double y_axis_point, double z_axis_point)
        {
            byte[] command = Encoding.UTF8.GetBytes("BC");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] motion_type = Encoding.UTF8.GetBytes("0");
            byte[] xy_type = Encoding.UTF8.GetBytes("1");

            decimal null_byte = 0;
            string null_st_fn = null_byte.ToString(ltn_stra2);

            decimal x_st = Convert.ToDecimal(x_axis_point);
            string x_st_fn = x_st.ToString(ltn_stra2);
            byte[] location_x = Encoding.UTF8.GetBytes(x_st_fn);
            byte[] location_null_x = Encoding.UTF8.GetBytes(null_st_fn);

            decimal y_st = Convert.ToDecimal(y_axis_point);
            string y_st_fn = y_st.ToString(ltn_stra2);
            byte[] location_y = Encoding.UTF8.GetBytes(y_st_fn);

            decimal z_st = Convert.ToDecimal(z_axis_point);
            string z_st_fn = z_st.ToString(ltn_stra2);
            byte[] location_z = Encoding.UTF8.GetBytes(z_st_fn);

            //byte[] xy_location_final = Combine(xy_location1, xy_location_null);

            //byte[] xy_location_final1 = Combine(xy_location2, xy_location3);
            //byte[] xy_location_final2 = Combine(xy_location3, xy_location4);

            //byte[] xy_location_final = Combine(xy_location1, xy_location_null);
            //xy_location_final = Combine(xy_location_final, xy_location2);
            //xy_location_final = Combine(xy_location_final, xy_location_null2);

            byte[] make_msg = Combine(command, channel_ba);
            make_msg = Combine(make_msg, motion_type);
            make_msg = Combine(make_msg, xy_type);

            make_msg = Combine(make_msg, location_x);
            make_msg = Combine(make_msg, location_y);
            make_msg = Combine(make_msg, location_z);
            make_msg = Combine(make_msg, location_z);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }


        public byte[] move_axis_all_y(string channel, double x_axis_point, double y_axis_point, double z_axis_point)
        {
            byte[] command = Encoding.UTF8.GetBytes("BC");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] motion_type = Encoding.UTF8.GetBytes("0");
            byte[] xy_type = Encoding.UTF8.GetBytes("1");

            decimal null_byte = 0;
            string null_st_fn = null_byte.ToString(ltn_stra2);
            ;
            decimal y_st = Convert.ToDecimal(y_axis_point);
            string y_st_fn = y_st.ToString(ltn_stra2);
            byte[] location_y = Encoding.UTF8.GetBytes(y_st_fn);
            //byte[] location_null_y = Encoding.UTF8.GetBytes(null_st_fn);

            decimal x_st = Convert.ToDecimal(x_axis_point);
            string x_st_fn = x_st.ToString(ltn_stra2);
            byte[] location_x = Encoding.UTF8.GetBytes(x_st_fn);

            decimal z_st = Convert.ToDecimal(z_axis_point);
            string z_st_fn = z_st.ToString(ltn_stra2);
            byte[] location_z = Encoding.UTF8.GetBytes(z_st_fn);


            //byte[] location_final = Combine(xy_location1, xy_location2);
            //byte[] xy_location_final1 = Combine(xy_location2, xy_location3);
            //byte[] location_final2 = Combine(xy_location3, xy_location4);

            //byte[] xy_location_final = Combine(xy_location1, xy_location_null);
            //xy_location_final = Combine(xy_location_final, xy_location2);
            //xy_location_final = Combine(xy_location_final, xy_location_null2);

            byte[] make_msg = Combine(command, channel_ba);
            make_msg = Combine(make_msg, motion_type);
            make_msg = Combine(make_msg, xy_type);

            make_msg = Combine(make_msg, location_x);
            make_msg = Combine(make_msg, location_y);
            make_msg = Combine(make_msg, location_z);
            make_msg = Combine(make_msg, location_z);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }
        public byte[] move_axis_all_z(string channel, double x_axis_point, double y_axis_point, double z_axis_point)
        {
            byte[] command = Encoding.UTF8.GetBytes("BC");
            byte[] channel_ba = Encoding.UTF8.GetBytes(channel);
            byte[] motion_type = Encoding.UTF8.GetBytes("0");
            byte[] xy_type = Encoding.UTF8.GetBytes("1");

            decimal null_byte = 0;
            string null_st_fn = null_byte.ToString(ltn_stra2);

            decimal x_st = Convert.ToDecimal(x_axis_point);
            string x_st_fn = x_st.ToString(ltn_stra2);
            byte[] location_x = Encoding.UTF8.GetBytes(x_st_fn);

            decimal y_st = Convert.ToDecimal(y_axis_point);
            string y_st_fn = y_st.ToString(ltn_stra2);
            byte[] location_y = Encoding.UTF8.GetBytes(y_st_fn);

            decimal z_st = Convert.ToDecimal(z_axis_point);
            string z_st_fn = z_st.ToString(ltn_stra2);
            byte[] location_z1 = Encoding.UTF8.GetBytes(z_st_fn);
            //byte[] location_null_z1 = Encoding.UTF8.GetBytes(null_st_fn);

            //byte[] location_z2 = Encoding.UTF8.GetBytes(z_st_fn);
            //byte[] location_null_z2 = Encoding.UTF8.GetBytes(null_st_fn);

            //byte[] xy_location_final = Combine(xy_location1, xy_location2);
            //byte[] xy_location_final1 = Combine(xy_location2, xy_location3);
            //byte[] xy_location_final2 = Combine(xy_location3, xy_location4);

            //byte[] xy_location_final = Combine(xy_location1, xy_location_null);
            //xy_location_final = Combine(xy_location_final, xy_location2);
            //xy_location_final = Combine(xy_location_final, xy_location_null2);

            byte[] make_msg = Combine(command, channel_ba);
            make_msg = Combine(make_msg, motion_type);
            make_msg = Combine(make_msg, xy_type);

            make_msg = Combine(make_msg, location_x);
            make_msg = Combine(make_msg, location_y);
            make_msg = Combine(make_msg, location_z1);
            make_msg = Combine(make_msg, location_z1);

            byte lrc = lrc_cal(make_msg);

            byte[] etx_ba = new byte[] { etx };
            byte[] lrc_ba = new byte[] { lrc };

            make_msg = Combine(make_msg, etx_ba);
            make_msg = Combine(make_msg, lrc_ba);

            return make_msg;
        }

        private enum RoundType
        {
            Ceiling,
            Round,
            Truncate
        }

        static private double CustomRound(RoundType roundType, double value, int digit = 1)
        {
            double dReturn = 0;

            // 지정 자릿수의 올림,반올림, 버림을 계산하기 위한 중간 계산
            double digitCal = Math.Pow(10, digit) / 10;

            switch (roundType)
            {
                case RoundType.Ceiling:
                    dReturn = Math.Ceiling(value * digitCal) / digitCal;
                    break;
                case RoundType.Round:
                    dReturn = Math.Round(value * digitCal) / digitCal;
                    break;
                case RoundType.Truncate:
                    dReturn = Math.Truncate(value * digitCal) / digitCal;
                    break;
            }
            return dReturn;
        }





        private void thorlabs_sh_connect_Click(object sender, EventArgs e)
        {
            open_time_sc10 = (Convert.ToInt16(exposure_time.Value) * 1000);


            string open_time_val = "open=" + Convert.ToString(open_time_sc10);

            // var task_shutter = Task.Run(() =>
            // {
            _serialPort.PortName = comboBox1.SelectedItem.ToString(); //"COM12";
            _serialPort.BaudRate = 9600;
            _serialPort.Parity = Parity.None;
            _serialPort.DataBits = 8;
            _serialPort.StopBits = StopBits.One;
            // Set the read/write timeouts
            _serialPort.ReadTimeout = -1; //1000
            _serialPort.WriteTimeout = -1;    //1000
            _serialPort.NewLine = "\r";

            _serialPort.Open();

            if (_serialPort.IsOpen == true)
            {
                Thread.Sleep(500);
                //_serialPort.WriteLine("\n");

                textBox5.Text = "Shutter Connected";
                thorlabs_sh_connect.Enabled = false;
                Thread.Sleep(500);

                _serialPort.WriteLine("mode=3");

                _serialPort.WriteLine(open_time_val);

                _serialPort.WriteLine("shut=1");
            }
            // });
        }

        private void button5_Click(object sender, EventArgs e)
        {
            _serialPort.WriteLine("mode=1");

            _serialPort.WriteLine("ens");
        }






    }
}
