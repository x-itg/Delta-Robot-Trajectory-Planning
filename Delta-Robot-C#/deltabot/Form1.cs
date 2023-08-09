using System;
using System.IO.Ports;
using System.Windows.Forms;
using System.Threading;
using System.Threading.Tasks;


namespace deltabot1_interface
{
 
    public partial class Form1 : Form
    {
        // robot geometry
        // (look at pics above for explanation)
        float ee = 46f;
        float ff = 142f;
        float re = 480f;
        float rf = 200f;

        // trigonometric constants
        //const Double sqrt3 = 1.732;
        const Double pi = 3.14159265358979;    // PI
        const Double sin120 = 0.8660254;
        const Double cos120 = -0.5;
        const Double tan60 = 1.7320508;
        const Double sin30 = 0.5;
        const Double tan30 = 0.57735;


        public Form1()
        {
            InitializeComponent();
        }

        private void button4_Click(object sender, EventArgs e)
        {

            // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
            // returned status: 0=OK, -1=non-existing position
            Double T1 = Convert.ToDouble(textBox5.Text);
            Double T2 = Convert.ToDouble(textBox6.Text);
            Double T3 = Convert.ToDouble(textBox7.Text);
            Double T12 = T1 ;
            Double T22 = T2 ;
            Double T32 = T3 ;
            Double X1 = 0;
            Double Y1 = 0;
            Double Z1 = 0;

            int Status1 = delta_calcForward(T1, T2, T3,ref X1,ref Y1,ref Z1);
            if (Status1 == 0)
            {
                textBox8.Text = "OK";
                textBox2.Text = Convert.ToString(X1);
                textBox3.Text = Convert.ToString(Y1);
                textBox4.Text = Convert.ToString(Z1);
            }
            else
            {
                textBox8.Text = "non-existing position";
                textBox2.Text = " Ø ";
                textBox3.Text = " Ø ";
                textBox4.Text = " Ø ";

            }

            

        }
        // 正解: 已知角度求位置(theta1, theta2, theta3) -> (x0, y0, z0)
        // 返回值: 0 = OK, -1 = 位置不存在
        int delta_calcForward(Double theta1, Double theta2, Double theta3, ref Double x0,ref Double y0,ref Double z0)
        {
            Double t = (ff - ee) * tan30 / 2;
            Double dtr = pi / 180.0;

            theta1 *= dtr;
            theta2 *= dtr;
            theta3 *= dtr;

            Double y1 = -(t + rf * Math.Cos(theta1));
            Double z1 = -rf * Math.Sin(theta1);

            Double y2 = (t + rf * Math.Cos(theta2)) * sin30;
            Double x2 = y2 * tan60;
            Double z2 = -rf * Math.Sin(theta2);

            Double y3 = (t + rf * Math.Cos(theta3)) * sin30;
            Double x3 = -y3 * tan60;
            Double z3 = -rf * Math.Sin(theta3);

            Double dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

            Double w1 = y1 * y1 + z1 * z1;
            Double w2 = x2 * x2 + y2 * y2 + z2 * z2;
            Double w3 = x3 * x3 + y3 * y3 + z3 * z3;

            // x = (a1*z + b1)/dnm
            Double a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
            Double b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

            // y = (a2*z + b2)/dnm;
            Double a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
            Double b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

            // a*z^2 + b*z + c = 0
            Double a = a1 * a1 + a2 * a2 + dnm * dnm;
            Double b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
            Double c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);

            // discriminant
            Double d = b * b - (Double)4.0 * a * c;
            if (d < 0)
            {
                return -1; // non-existing point
            }

            z0 = -0.5 * (b + Math.Sqrt(d)) / a;
            x0 = (a1 * z0 + b1) / dnm;
            y0 = (a2 * z0 + b2) / dnm;
            return 0;
        }
        private void button3_Click(object sender, EventArgs e)
        {
            ee = float.Parse(tbx_ee.Text);
            ff = float.Parse(tbx_ff.Text);
            re = float.Parse(tbx_re.Text);
            rf = float.Parse(tbx_rf.Text);


            Double X1 = Convert.ToDouble(textBox2.Text);
            Double Y1 = Convert.ToDouble(textBox3.Text);
            Double Z1 = Convert.ToDouble(textBox4.Text);
            Double T1 = 0;
            Double T2 = 0;
            Double T3 = 0;
          

            int Status2 = delta_calcInverse(X1, Y1, Z1, ref T1, ref T2, ref T3);
            if (Status2 == 0)
            {
                textBox8.Text = "OK";
                textBox5.Text = Convert.ToString(T1);
                textBox6.Text = Convert.ToString(T2);
                textBox7.Text = Convert.ToString(T3);
            }
            else
            {
                textBox8.Text = "non-existing position";
                textBox5.Text = " Ø ";
                textBox6.Text = " Ø ";
                textBox7.Text = " Ø ";
            }




        }
        // 逆解:已知坐标求角度 (x0, y0, z0) -> (theta1, theta2, theta3)
        // 返回值: 0=OK, -1 = 位置不存在
        int delta_calcInverse(Double x0, Double y0, Double z0, ref Double theta1, ref Double theta2, ref Double theta3)
        {
            theta1 = theta2 = theta3 = 0;
            int status = delta_calcAngleYZ(x0, y0, z0, ref theta1);
            if (status == 0) status = delta_calcAngleYZ(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0, ref theta2);  // rotate coords to +120 deg
            if (status == 0) status = delta_calcAngleYZ(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0, ref theta3);  // rotate coords to -120 deg
            return status;
        }
        // 帮助函数, 计算角度 theta1 (for YZ-pane)
        int delta_calcAngleYZ(Double x0, Double y0, Double z0,ref Double theta)
        {
            

            Double y1 = -0.5 * 0.57735 * ff; // f/2 * tg 30
            y0 -= 0.5 * 0.57735 * ee;    // shift center to edge
                                         // z = a + b*y
            Double a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
            Double b = (y1 - y0) / z0;
            // discriminant
            Double d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
            if (d < 0) return -1; // non-existing point
            Double yj = (y1 - a * b - Math.Sqrt(d)) / (b * b + 1); // choosing outer point
            Double zj = a + b * yj;
            theta = 180.0 * Math.Atan(-zj / (y1 - yj)) / pi + ((yj > y1) ? 180.0 : 0.0);
            return 0;
        }




    }
}
