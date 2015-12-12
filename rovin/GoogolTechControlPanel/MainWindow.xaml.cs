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
                PipeServer.SendMessage("sx " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void sy_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                PipeServer.SendMessage("sy " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void sz_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                PipeServer.SendMessage("sz " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void ex_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                PipeServer.SendMessage("ex " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void ey_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                PipeServer.SendMessage("ey " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void ez_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                PipeServer.SendMessage("ez " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void sy_Copy_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
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
                PipeServer.SendMessage("st " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void salpha_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                PipeServer.SendMessage("sa " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void etheta_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                PipeServer.SendMessage("et " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void ealpha_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (start)
            {
                PipeServer.SendMessage("ea " + ((Slider)sender).Value + "=", PipeServer.clientse);
            }
        }

        private void energyButton_Checked(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                PipeServer.SendMessage("effort=", PipeServer.clientse);
            }
        }

        private void effortButton_Checked(object sender, RoutedEventArgs e)
        {
            if (start)
            {
                PipeServer.SendMessage("energyloss=", PipeServer.clientse);
            }
        }
    }
}
