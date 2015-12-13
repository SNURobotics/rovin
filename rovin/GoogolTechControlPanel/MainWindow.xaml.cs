using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace GoogolTechControlPanel
{
    /// <summary>
    /// MainWindow.xaml에 대한 상호 작용 논리
    /// </summary>
    public partial class MainWindow : Window
    {
        NamedPipeServer PipeServer;
        bool start = false;

        public MainWindow()
        {
            InitializeComponent();

            PipeServer = new NamedPipeServer(@"\\.\pipe\NamePipeGoogolTech", 1);
            PipeServer.Start();
            start = true;
        }

        protected override void OnClosed(EventArgs e)
        {
            PipeServer.StopServer();
            base.OnClosed(e);
        }

        private void sx_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if(start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("sx " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void sy_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("sy " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void sz_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("sz " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void ex_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("ex " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void ey_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("ey " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void ez_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("ez " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void sy_Copy_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("tf " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void applyButton_Click(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                PipeServer.SendMessage("apply=", PipeServer.clientse);
            }
        }

        private void stheta_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("st " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void salpha_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("sa " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void etheta_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("et " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void ealpha_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("ea " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void energyButton_Checked(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("energyloss=", PipeServer.clientse);
            }
        }

        private void effortButton_Checked(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("effort=", PipeServer.clientse);
            }
        }

        private void w1c_Checked(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w1c=", PipeServer.clientse);
                PipeServer.SendMessage("w1x " + w1x.Value + "=", PipeServer.clientse);
                PipeServer.SendMessage("w1y " + w1y.Value + "=", PipeServer.clientse);
                PipeServer.SendMessage("w1z " + w1z.Value + "=", PipeServer.clientse);
                PipeServer.SendMessage("w1t " + w1t.Value + "=", PipeServer.clientse);
                PipeServer.SendMessage("w1a " + w1a.Value + "=", PipeServer.clientse);
                PipeServer.SendMessage("w1time " + w1time.Value + "=", PipeServer.clientse);
            }
        }

        private void w1c_Unchecked(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w1uc=", PipeServer.clientse);
            }
        }

        private void w1x_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w1x " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void w1y_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w1y " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void w1z_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w1z " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void w1t_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w1t " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void w1a_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w1a " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void w1time_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w1time " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void w2x_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w2x " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void w2y_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w2y " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void w2z_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w2z " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void w2t_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w2t " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void w2a_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w2a " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void w2time_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w2time " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void timeconstraint_Checked(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("timec=", PipeServer.clientse);
            }
        }

        private void timeconstraint_unChecked(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("timeuc=", PipeServer.clientse);
            }
        }

        private void w2c_Unchecked_1(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w2uc=", PipeServer.clientse);
            }
        }

        private void w2c_Checked(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("w2c=", PipeServer.clientse);
                PipeServer.SendMessage("w2x " + w2x.Value + "=", PipeServer.clientse);
                PipeServer.SendMessage("w2y " + w2y.Value + "=", PipeServer.clientse);
                PipeServer.SendMessage("w2z " + w2z.Value + "=", PipeServer.clientse);
                PipeServer.SendMessage("w2t " + w2t.Value + "=", PipeServer.clientse);
                PipeServer.SendMessage("w2a " + w2a.Value + "=", PipeServer.clientse);
                PipeServer.SendMessage("w2time " + w2time.Value + "=", PipeServer.clientse);
            }
        }

        private void allsend()
        {
            string message = "";
            message += "sx " + sx.Value + "=";
            message += "sy " + sy.Value + "=";
            message += "sz " + sz.Value + "=";
            message += "ex " + ex.Value + "=";
            message += "ey " + ey.Value + "=";
            message += "ez " + ez.Value + "=";
            message += "tf " + finaltime.Value + "=";
            message += "st " + stheta.Value + "=";
            message += "sa " + salpha.Value + "=";
            message += "et " + etheta.Value + "=";
            message += "ea " + ealpha.Value + "=";
            message += "w1x " + w1x.Value + "=";
            message += "w1y " + w1y.Value + "=";
            message += "w1z " + w1z.Value + "=";
            message += "w1t " + w1t.Value + "=";
            message += "w1a " + w1a.Value + "=";
            message += "w1time " + w1time.Value + "=";
            message += "w2x " + w2x.Value + "=";
            message += "w2y " + w2y.Value + "=";
            message += "w2z " + w2z.Value + "=";
            message += "w2t " + w2t.Value + "=";
            message += "w2a " + w2a.Value + "=";
            message += "w2time " + w2time.Value + "=";
            message += "ew " + weight.Value + "=";

            if (effortButton.IsChecked == true)
            {
                message += "effort=";
            }
            else
            {
                message += "energyloss=";
            }

            if (w1c.IsChecked == true)
            {
                message += "w1c=";
            }
            else
            {
                message += "w1uc=";
            }

            if (w2c.IsChecked == true)
            {
                message += "w2c=";
            }
            else
            {
                message += "w2uc=";
            }

            if (wrist.IsChecked == true)
            {
                message += "wrist=";
            }
            else
            {
                message += "uwrist=";
            }

            if (timeconstraint.IsChecked == true)
            {
                message += "timec=";
            }
            else
            {
                message += "timeuc=";
            }

            PipeServer.SendMessage(message, PipeServer.clientse);
        }

        private void apply_Click(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                allsend();
            }
        }

        private void ex1_Click(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                sx.Value = 70;
                sy.Value = 90;
                sz.Value = 5;
                ex.Value = 60;
                ey.Value = 5;
                ez.Value = 80;
                finaltime.Value = 16;
                stheta.Value = 70;
                salpha.Value = 40;
                etheta.Value = 10;
                ealpha.Value = 10;
                w1x.Value = 50;
                w1y.Value = 50;
                w1z.Value = 0;
                w1t.Value = 50;
                w1a.Value = 50;
                w1time.Value = 33;
                w2x.Value = 50;
                w2y.Value = 50;
                w2z.Value = 0;
                w2t.Value = 50;
                w2a.Value = 50;
                w2time.Value = 66;
                weight.Value = 0;
                effortButton.IsChecked = true;
                energyButton.IsChecked = false;
                timeconstraint.IsChecked = false;
                w1c.IsChecked = false;
                w2c.IsChecked = false;
                wrist.IsChecked = false;
                allsend();
            }
        }

        private void ex2_Click(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                sx.Value = 25.8387;
                sy.Value = 92.1505;
                sz.Value = 32.9569;
                ex.Value = 60;
                ey.Value = 14.6774;
                ez.Value = 23.655;
                finaltime.Value = 24.0;
                stheta.Value = 70;
                salpha.Value = 40;
                etheta.Value = 10;
                ealpha.Value = 10;
                w1x.Value = 50;
                w1y.Value = 50;
                w1z.Value = 0;
                w1t.Value = 50;
                w1a.Value = 50;
                w1time.Value = 33;
                w2x.Value = 50;
                w2y.Value = 50;
                w2z.Value = 0;
                w2t.Value = 50;
                w2a.Value = 50;
                w2time.Value = 66;
                weight.Value = 100;
                effortButton.IsChecked = true;
                energyButton.IsChecked = false;
                timeconstraint.IsChecked = false;
                w1c.IsChecked = false;
                w2c.IsChecked = false;
                wrist.IsChecked = true;
                allsend();
            }
        }

        private void ex3_Click(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                sx.Value = 81.8666419354839;
                sy.Value = 50;
                sz.Value = 25.6343946236559;
                ex.Value = 50;
                ey.Value = 0;
                ez.Value = 46.3118;
                finaltime.Value = 18.48187;
                stheta.Value = 50;
                salpha.Value = 0;
                etheta.Value = 25.2688;
                ealpha.Value = 50;
                w1x.Value = 50;
                w1y.Value = 8.57526881720434;
                w1z.Value = 46.3118;
                w1t.Value = 50;
                w1a.Value = 50;
                w1time.Value = 33;
                w2x.Value = 50;
                w2y.Value = 50;
                w2z.Value = 0;
                w2t.Value = 50;
                w2a.Value = 50;
                w2time.Value = 66;
                weight.Value = 100;
                effortButton.IsChecked = true;
                energyButton.IsChecked = false;
                timeconstraint.IsChecked = false;
                w1c.IsChecked = false;
                w2c.IsChecked = false;
                wrist.IsChecked = true;
                allsend();
            }
        }

        private void ex4_Click(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                sx.Value = 44.193;
                sy.Value = 90;
                sz.Value = 5;
                ex.Value = 60;
                ey.Value = 5;
                ez.Value = 80;
                finaltime.Value = 21.65;
                stheta.Value = 70;
                salpha.Value = 40;
                etheta.Value = 10;
                ealpha.Value = 10;
                w1x.Value = 78;
                w1y.Value = 73.1118;
                w1z.Value = 27.1075;
                w1t.Value = 50;
                w1a.Value = 50;
                w1time.Value = 33;
                w2x.Value = 50;
                w2y.Value = 50;
                w2z.Value = 0;
                w2t.Value = 50;
                w2a.Value = 50;
                w2time.Value = 66;
                weight.Value = 100;
                effortButton.IsChecked = true;
                energyButton.IsChecked = false;
                timeconstraint.IsChecked = false;
                w1c.IsChecked = false;
                w2c.IsChecked = false;
                wrist.IsChecked = false;
                allsend();
            }
        }

        private void ex5_Click(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                sx.Value = 44.193;
                sy.Value = 90;
                sz.Value = 5;
                ex.Value = 77.204;
                ey.Value = 35.107;
                ez.Value = 13.333;
                finaltime.Value = 18.2939;
                stheta.Value = 70;
                salpha.Value = 40;
                etheta.Value = 10;
                ealpha.Value = 10;
                w1x.Value = 78;
                w1y.Value = 73.1118;
                w1z.Value = 27.1075;
                w1t.Value = 50;
                w1a.Value = 50;
                w1time.Value = 33;
                w2x.Value = 50;
                w2y.Value = 50;
                w2z.Value = 0;
                w2t.Value = 50;
                w2a.Value = 50;
                w2time.Value = 66;
                weight.Value = 100;
                effortButton.IsChecked = true;
                energyButton.IsChecked = false;
                timeconstraint.IsChecked = false;
                w1c.IsChecked = false;
                w2c.IsChecked = false;
                wrist.IsChecked = true;
                allsend();
            }
        }

        private void ex6_Click(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                sx.Value = 85.4838;
                sy.Value = 80.1075;
                sz.Value = 5;
                ex.Value = 85.4838;
                ey.Value = 20.9677;
                ez.Value = 5;
                finaltime.Value = 25.1149;
                stheta.Value = 50;
                salpha.Value = 0;
                etheta.Value = 50;
                ealpha.Value = 0;
                w1x.Value = 85.4838;
                w1y.Value = 77.1075;
                w1z.Value = 16.1290;
                w1t.Value = 50;
                w1a.Value = 50;
                w1time.Value = 33;
                w2x.Value = 85.4838;
                w2y.Value = 23.9677;
                w2z.Value = 16.1290;
                w2t.Value = 50;
                w2a.Value = 50;
                w2time.Value = 66;
                weight.Value = 100;
                effortButton.IsChecked = false;
                energyButton.IsChecked = true;
                timeconstraint.IsChecked = false;
                w1c.IsChecked = true;
                w2c.IsChecked = true;
                wrist.IsChecked = false;
                allsend();
            }
        }

        private void weight_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("ew " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void checkBox_Checked(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("wrist=", PipeServer.clientse);
            }
        }

        private void checkBox_Unchecked(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                if (PipeServer.clientse == null) return;
                PipeServer.SendMessage("uwrist=", PipeServer.clientse);
            }
        }
    }
}
