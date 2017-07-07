using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace SimuladorMotorCD
{
    public partial class Form1 : Form
    {
        double k1, k2, k3, k4;
        double[] i = new double[1000];
        double []Eb = new double[1000];
        double []Tt = new double[1000];
        double[] w = new double[1000];
        int count;
        double prop, inte, deri;
        double CO, COSat;
        double COAnt;
        double error;
        double errorAnt = 0;

        public Form1()
        {
            InitializeComponent();
            numericUpDownEa.Value = 0;
            numericUpDownRa.Value = (decimal)0.316;
            numericUpDownLa.Value = (decimal)0.082;
            numericUpDownKt.Value = (decimal)30.2;
            numericUpDownKb.Value = (decimal)317;
            numericUpDownJ.Value = (decimal)0.139;
            numericUpDownB.Value = (decimal)0;
            numericUpDownH.Value = (decimal)0.0005;
        }

        private void label2_Click(object sender, EventArgs e)
        {

        }

        private void Form1_Load(object sender, EventArgs e)
        {
            i[0] = 0;
            Eb[0] = 0;
            Tt[0] = 0;
            w[0] = 0;
            inte = 0;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            //timer1.Start();

            for (count = 1; count <= 1000; count++)
            {
                calculos();

                if((count -1)%10 == 0)
                    PID();
            }
        }

        private void numericUpDownH_ValueChanged(object sender, EventArgs e)
        {
           
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            count++;
            calculos();
            PID();
        }

        private void PID()
        {
            double sp = (double)numericUpDownSp.Value * (2*Math.PI) / 60;
            double kp = (double)numericUpDownKp.Value;
            double ki = (double)numericUpDownKi.Value;
            double kd = (double)numericUpDownKd.Value;
            double pv = w[999];

            error = sp - pv;
            
            prop = kp * error;
            deri = kd * (error - errorAnt);

            CO += prop + inte + deri;

            if (CO > 24)
                COSat = 24;
            else if (CO < 0)
                COSat = 0;
            else
                COSat = CO;

            numericUpDownEa.Value = (decimal)COSat;

            inte = ki * (0.02 * (error + (COSat - CO)) + inte);

            CO = COSat;
            errorAnt = error;
        }

        private void calculos()
        {
            double iAct, iSig;
            double wAct, wSig;
            double x, y;
            
            //Lectura de datos de numericUpDown
            double h = (double)(numericUpDownH.Value);
            double La = (double)numericUpDownLa.Value / 1000;
            double Ea = (double)numericUpDownEa.Value;
            double Ra = (double)numericUpDownRa.Value;
            double kt = (double)numericUpDownKt.Value / 1000;
            double kb = (double)(numericUpDownKb.Value) * (double)(1.0 / 60.0) * (double)(2 * Math.PI);
            kb = 1 / kb;
            double J = (double)numericUpDownJ.Value / 1000;
            double B = (double)numericUpDownB.Value;
            double load = (double)numericUpDownLoad.Value;

            //Calculo de corriente por runge kutta
            iAct = i[999];
            k1 = h * (1 / La) * (Ea - (Ra * i[999]) - Eb[999]);
            k2 = h * (1 / La) * (Ea - (Ra * (i[999] + k1 / 2)) - Eb[999]);
            k3 = h * (1 / La) * (Ea - (Ra * (i[999] + k2 / 2)) - Eb[999]);
            k4 = h * (1 / La) * (Ea - (Ra * (i[999] + k3)) - Eb[999]);
            iSig = iAct + (1.0/6.0)*(k1 + (2*k2) + (2*k3) + k4);

            //calculo de torque de motor
            Tt[999] = iAct * kt;

            //calculo de velocidad angular por runge kutta
            wAct = w[999-1];
            k1 = h * (1 / J) * (Tt[999] - (B * w[999]) - load);
            k2 = h * (1 / J) * (Tt[999] - (B * (w[999] + k1 / 2)) - load);
            k3 = h * (1 / J) * (Tt[999] - (B * (w[999] + k2 / 2)) - load);
            k4 = h * (1 / J) * (Tt[999] - (B * (w[999] + k3)) - load);
            wSig = wAct + (1.0 / 6.0) * (k1 + (2 * k2) + (2 * k3) + k4);
            w[999] = wSig;

            //Calculo de FEM
            Eb[999] = (kb * wAct);

            for (int j = 0; j < 999; j++)
            {
                i[j] = i[j + 1];
                w[j] = w[j + 1];
            }
            i[999] = iSig;
            w[999] = wSig;

            impresionResultados();

            /*listBox1.Items.Add(h*count
                                + "\t" + Ea + "V"
                                + "\t" + iAct + "A"
                                + "\t" + Ra + "ohm"
                                + "\t" + La + "mH"
                                + "\t" + Eb[999] + "V"
                                + "\t" + Tt[999] + "Nm"
                                + "\t" + kt + "Nm/A"
                                + "\t" + kb + "V/rad/s"
                                + "\t" + wAct + "rad/s" 
                                );
             */
 
            listBox1.Items.Add(h*count
                                + "\t" + Ea + "V"
                                + "\t \t" + error
                                + "\t \t" + CO);
        }

        private void impresionResultados ()
        {
            double corriente;
            double velAngular;
            double setPoint;
            double t;

            chart1.Series["Corriente (A)"].Points.Clear();
            chart2.Series["Velocidad Angular (RPM)"].Points.Clear();
            chart2.Series["SetPoint"].Points.Clear();

            setPoint = (double)numericUpDownSp.Value;

            for (int k = 0; k < 1000; k++)
            {
                t = (double)(numericUpDownH.Value * (k-999+count));
                corriente = i[k];
                velAngular = w[k] * 60 / (2 * Math.PI);
                
                chart1.Series["Corriente (A)"].Points.AddXY(t, corriente);
                chart2.Series["Velocidad Angular (RPM)"].Points.AddXY(t, velAngular);
                chart2.Series["SetPoint"].Points.AddXY(t, setPoint);
            }
        }

        private void buttonRestart_Click(object sender, EventArgs e)
        {
            count = 0;
            chart1.Series["Corriente (A)"].Points.Clear();
            chart2.Series["Velocidad Angular (RPM)"].Points.Clear();
            chart2.Series["SetPoint"].Points.Clear();

            listBox1.Items.Clear();

            numericUpDownEa.Value = 0;

            for (int k = 0; k < 1000; k++)
            {
                i[k] = 0;
                w[k] = 0;
                Eb[k] = 0;
                Tt[k] = 0;
            }
            errorAnt = 0;
            CO = 0;
        }

        private void chart1_Click(object sender, EventArgs e)
        {

        }

        private void buttonStop_Click(object sender, EventArgs e)
        {
            timer1.Stop();
        }

        private void chart2_Click(object sender, EventArgs e)
        {

        }
    }
}
