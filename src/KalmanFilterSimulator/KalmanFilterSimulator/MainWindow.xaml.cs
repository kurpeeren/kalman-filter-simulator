using System;
using System.Data;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Shapes;
using System.Windows.Threading;
using MathNet.Numerics.LinearAlgebra;

namespace KalmanFilterSimulator
{
    public partial class MainWindow : Window
    {
        private Vector<double> x;
        private Matrix<double> P;
        private readonly Matrix<double> F;
        private readonly Matrix<double> Q;
        private readonly Matrix<double> H;
        private readonly Matrix<double> R;

        private Random random = new Random();
        private Thread simulationThread;
        private Dispatcher mainDispatcher; // Ana iş parçacığı
        private volatile bool isRunning = true; // İş parçacığı kontrol işaretçisi




        public MainWindow()
        {
            InitializeComponent();
            // Initialize TransformGroup
            transformGroup.Children.Add(scaleTransform);
            transformGroup.Children.Add(translateTransform);

            // Apply the TransformGroup to the canvas
            canvas.RenderTransform = transformGroup;
            mainDispatcher = Dispatcher.CurrentDispatcher; // Ana iş parçacığını kaydet
            StartSimulationTask();

            canvas.MouseWheel += Canvas_MouseWheel;
            // Initialize Kalman Filter parameters
            x = Vector<double>.Build.Dense(new double[] { 0, 0, 0, 0 });
            P = Matrix<double>.Build.DenseIdentity(4) * 1000;
            F = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                { 1, 0, 0.1, 0 },
                { 0, 1, 0, 0.1 },
                { 0, 0, 1, 0 },
                { 0, 0, 0, 1 }
            });
            Q = Matrix<double>.Build.DenseIdentity(4) * 0.1;
            H = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                { 1, 0, 0, 0 },
                { 0, 1, 0, 0 }
            });
            R = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                { 5, 0 },
                { 0, 5 }
            });

        }
        int time=10;
        private CancellationTokenSource cts;
        int variance1 = 100;
        int variance2 = 50;
        // Sensör sayaçları ve periyotlar
        private int sensor1Counter = 0;
        private int sensor2Counter = 0;

        private int sensor1Period = 10; // Sensor1 her 10 döngüde bir veri üretecek
        private int sensor2Period = 10;  // Sensor2 her döngüde veri üretecek


        // SPS (samples per second) değeri
        double sensor1SPS = 10; // Sensör 1 için 10 örnek/saniye
        double sensor2SPS = 25;  // Sensör 2 için 25 örnek/saniye



        public void StartSimulationTask()
        {
            cts = new CancellationTokenSource();
            var token = cts.Token;

            // Son sensör güncelleme zamanları
            DateTime lastSensor1Update = DateTime.Now;
            DateTime lastSensor2Update = DateTime.Now;

            // Sensör son ölçüm değerleri
            Point mousePos = Mouse.GetPosition(canvas);
            Vector<double> sensor1Last = Vector<double>.Build.Dense(new double[] { mousePos.X, mousePos.Y });
            Vector<double> sensor2Last = Vector<double>.Build.Dense(new double[] { mousePos.X, mousePos.Y });

            Task.Run(() =>
            {
                Random random = new Random();

                // H matrisi
                var H_extended = Matrix<double>.Build.DenseOfArray(new double[,]
                {
                    { 1, 0, 0, 0 },  // Gerçek pozisyon x
                    { 0, 1, 0, 0 },  // Gerçek pozisyon y
                    { 1, 0, 0, 0 },  // Sensor1 pozisyon x
                    { 0, 1, 0, 0 },  // Sensor1 pozisyon y
                    { 1, 0, 0, 0 },  // Sensor2 pozisyon x
                    { 0, 1, 0, 0 }   // Sensor2 pozisyon y
                });

                // R matrisi (ilk başta sabit)
                var R_extended = Matrix<double>.Build.DenseOfArray(new double[,]
                {
                    { 5, 0, 0, 0, 0, 0 },
                    { 0, 5, 0, 0, 0, 0 },
                    { 0, 0, 5, 0, 0, 0 },
                    { 0, 0, 0, 5, 0, 0 },
                    { 0, 0, 0, 0, 5, 0 },
                    { 0, 0, 0, 0, 0, 5 }
                });

                // Ölçüm verileri listeleri
                List<double> sensor1XMeasurements = new List<double>();
                List<double> sensor1YMeasurements = new List<double>();
                List<double> sensor2XMeasurements = new List<double>();
                List<double> sensor2YMeasurements = new List<double>();

                while (!token.IsCancellationRequested)
                {
                    Application.Current.Dispatcher.Invoke(() =>
                    {
                        if (Mouse.LeftButton == MouseButtonState.Pressed)
                        {

                            // Sensör yenileme periyotları
                            int sensor1PeriodMs = (int)(1000 / sensor1SPS);
                            int sensor2PeriodMs = (int)(1000 / sensor2SPS);

                            // Sensör aktiflik kontrolleri
                            H_extended[0, 0] = ActualMouseActive.IsChecked.Value ? 1 : 0;
                            H_extended[1, 1] = ActualMouseActive.IsChecked.Value ? 1 : 0;

                            H_extended[2, 0] = sensor1Active.IsChecked.Value ? 1 : 0;
                            H_extended[3, 1] = sensor1Active.IsChecked.Value ? 1 : 0;

                            H_extended[4, 0] = sensor2Active.IsChecked.Value ? 1 : 0;
                            H_extended[5, 1] = sensor2Active.IsChecked.Value ? 1 : 0;


                            //R_extended[2, 2] = Convert.ToDouble(s1X.Text == "" ? 0 : s1X.Text);
                            //R_extended[3, 3] = Convert.ToDouble(s1Y.Text == "" ? 0 : s1Y.Text);
                            //R_extended[4, 4] = Convert.ToDouble(s2X.Text == "" ? 0 : s2X.Text);
                            //R_extended[5, 5] = Convert.ToDouble(s2Y.Text == "" ? 0 : s2Y.Text);

                            // Mouse pozisyonu
                            Point mousePos = Mouse.GetPosition(canvas);
                            var z_actual = Vector<double>.Build.Dense(new double[] { mousePos.X, mousePos.Y });

                            // Sensör 1 ölçümü
                            Vector<double> sensor1;
                            DateTime now = DateTime.Now;

                            if ((now - lastSensor1Update).TotalMilliseconds >= sensor1PeriodMs)
                            {
                                double noiseX1 = GenerateGaussianNoise(0, variance1);
                                double noiseY1 = GenerateGaussianNoise(0, variance1);
                                sensor1 = Vector<double>.Build.Dense(new double[] { mousePos.X + noiseX1, mousePos.Y + noiseY1 });
                                sensor1Last = sensor1;

                                // Ölçüm verilerini kaydet
                                sensor1XMeasurements.Add(sensor1[0]);
                                sensor1YMeasurements.Add(sensor1[1]);

                                lastSensor1Update = now;
                            }
                            else
                            {
                                sensor1 = sensor1Last;
                            }

                            // Sensör 2 ölçümü
                            Vector<double> sensor2;
                            if ((now - lastSensor2Update).TotalMilliseconds >= sensor2PeriodMs)
                            {
                                double noiseX2 = GenerateGaussianNoise(0, variance2);
                                double noiseY2 = GenerateGaussianNoise(0, variance2);
                                sensor2 = Vector<double>.Build.Dense(new double[] { mousePos.X + noiseX2, mousePos.Y + noiseY2 });
                                sensor2Last = sensor2;

                                // Ölçüm verilerini kaydet
                                sensor2XMeasurements.Add(sensor2[0]);
                                sensor2YMeasurements.Add(sensor2[1]);

                                lastSensor2Update = now;
                            }
                            else
                            {
                                sensor2 = sensor2Last;
                            }

                            // Ölçüm vektörü
                            var z_combined = Vector<double>.Build.Dense(new double[]
                            {
                        ActualMouseActive.IsChecked.Value ? z_actual[0] : 0,
                        ActualMouseActive.IsChecked.Value ? z_actual[1] : 0,
                        sensor1[0],
                        sensor1[1],
                        sensor2[0],
                        sensor2[1]
                            });

                            // R_extended matrisini güncelle
                            if (dynamicR.IsChecked.Value) { 
                                R_extended[2, 2] = CalculateVariance(sensor1XMeasurements, z_actual[0]);
                                R_extended[3, 3] = CalculateVariance(sensor1YMeasurements, z_actual[1]);
                                R_extended[4, 4] = CalculateVariance(sensor2XMeasurements, z_actual[0]);
                                R_extended[5, 5] = CalculateVariance(sensor2YMeasurements, z_actual[1]);
                                //s1X.Content = R_extended[2, 2].ToString();
                                //s1Y.Content = R_extended[3, 3].ToString();
                                //s2X.Content = R_extended[4, 4].ToString();
                                //s2Y.Content = R_extended[5, 5].ToString();
                            }

                            // Kalman filtresi adımları
                            (x, P) = KalmanFilter.Predict(x, P, F, Q);
                            (x, P) = KalmanFilter.Update(x, P, H_extended, R_extended, z_combined);

                            // Noktaları çiz
                            if (plot_sensor1.IsChecked.Value) DrawPoint(sensor1[0], sensor1[1], Brushes.Blue);
                            if (plot_sensor2.IsChecked.Value) DrawPoint(sensor2[0], sensor2[1], Brushes.Orange);
                            if (plot_mouse.IsChecked.Value)   DrawPoint(z_actual[0], z_actual[1], Brushes.Green);
                            if (plot_kalman.IsChecked.Value)  DrawPoint(x[0], x[1], Brushes.Red);
                        }

                        // Z tuşuna basıldığında canvas temizlenir
                        if (Keyboard.IsKeyDown(Key.Z))
                        {
                            canvas.Children.Clear();
                        }
                    });

                    Thread.Sleep(1); // CPU yükünü azaltmak için
                }
            }, token);
        }
        private DataTable ConvertMatrixToDataTable(Matrix<double> matrix)
        {
            DataTable table = new DataTable();

            // Kolon başlıklarını ekle
            for (int col = 0; col < matrix.ColumnCount; col++)
            {
                table.Columns.Add($"Col {col + 1}");
            }

            // Satırları ekle
            for (int row = 0; row < matrix.RowCount; row++)
            {
                DataRow dataRow = table.NewRow();
                for (int col = 0; col < matrix.ColumnCount; col++)
                {
                    dataRow[col] = matrix[row, col];
                }
                table.Rows.Add(dataRow);
            }

            return table;
        }

        // Varyans hesaplama metodu
        private double CalculateVariance(List<double> measurements, double trueValue)
        {
            if (measurements.Count == 0) return 0; // Bölme hatasını önlemek için
            double sumSquaredErrors = measurements.Select(m => Math.Pow(m - trueValue, 2)).Sum();
            return sumSquaredErrors / measurements.Count;
        }


        private double GenerateGaussianNoise(double mean, double variance)
        {
            Random random = new Random();
            double u1 = random.NextDouble();
            double u2 = random.NextDouble();
            double standardNormal = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
            return mean + Math.Sqrt(variance) * standardNormal;
        }


        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (cts != null)
            {
                cts.Cancel();
                cts.Dispose();
            }
        }

        private void DrawPoint(double x, double y, Brush color)
        {
            Ellipse ellipse = new Ellipse
            {
                Width = 5,
                Height = 5,
                Fill = color
            };

            Canvas.SetLeft(ellipse, x);
            Canvas.SetTop(ellipse, y);
            canvas.Children.Add(ellipse);
        }
        private double _scale = 1.0; // Başlangıç ölçeği
        private TranslateTransform translateTransform = new TranslateTransform();
        private ScaleTransform scaleTransform = new ScaleTransform();
        private TransformGroup transformGroup = new TransformGroup();

        private void Canvas_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            Point mousePosition = e.GetPosition(canvas);

            // CTRL key pressed for zoom
            if (Keyboard.IsKeyDown(Key.LeftCtrl) || Keyboard.IsKeyDown(Key.RightCtrl))
            {
                double zoomFactor = e.Delta > 0 ? 1.1 : 0.9; // Zoom in or out
                _scale *= zoomFactor;

                // Adjust TranslateTransform to center zoom on the mouse position
                double offsetX = mousePosition.X - (mousePosition.X - translateTransform.X) * zoomFactor;
                double offsetY = mousePosition.Y - (mousePosition.Y - translateTransform.Y) * zoomFactor;

                scaleTransform.ScaleX = _scale;
                scaleTransform.ScaleY = _scale;
                translateTransform.X = offsetX;
                translateTransform.Y = offsetY;

                e.Handled = true;
            }
            // SHIFT tuşuna basılı ise yatay kaydırma
            if (Keyboard.IsKeyDown(Key.LeftShift) || Keyboard.IsKeyDown(Key.RightShift))
            {
                if (e.Delta > 0) // Fare yukarı hareket ederse sola kaydır
                {
                    ScrollViewer.ScrollToHorizontalOffset(ScrollViewer.HorizontalOffset - 20);
                }
                else // Fare aşağı hareket ederse sağa kaydır
                {
                    ScrollViewer.ScrollToHorizontalOffset(ScrollViewer.HorizontalOffset + 20);
                }

                e.Handled = true;
            }
        }


        public static class KalmanFilter
        {
            public static (Vector<double>, Matrix<double>) Predict(Vector<double> x, Matrix<double> P, Matrix<double> F, Matrix<double> Q)
            {
                // Durum tahmini
                x = F * x;

                // Kovaryans tahmini
                P = F * P * F.Transpose() + Q;

                return (x, P);
            }

            public static (Vector<double>, Matrix<double>) Update(Vector<double> x, Matrix<double> P, Matrix<double> H, Matrix<double> R, Vector<double> z)
            {
                // Gözlemler ile tahmin edilen durum arasındaki farktır
                var y = z - H * x;

                // Bu değer, ölçüm hatası kovaryansı (R) ile projeksiyon kovaryansının toplamını ifade eder
                var S = H * P * H.Transpose() + R;

                // Kalman kazancı
                var K = P * H.Transpose() * S.Inverse();

                // Durum tahmininin güncellenmesi
                x = x + K * y;

                // Kovaryans tahmininin güncellenmesi
                P = P - K * H * P;

                return (x, P);
            }
        }


        private void Canvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {

        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {

        }

        private void refleshTime_TextChanged(object sender, TextChangedEventArgs e)
        {
            time=Convert.ToInt32( refleshTime.Text);
        }

        private void Window_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.R) // Reset on R key
            {
                _scale = 1.0;
                scaleTransform.ScaleX = 1.0;
                scaleTransform.ScaleY = 1.0;
                translateTransform.X = 0;
                translateTransform.Y = 0;

            }
        }
        private void hata1_TextChanged(object sender, TextChangedEventArgs e)
        {
            variance1 = Convert.ToInt32(hata1.Text==""? "0" : hata1.Text);

        }
        private void hata2_TextChanged(object sender, TextChangedEventArgs e)
        {
            variance2 = Convert.ToInt32(hata2.Text == "" ? "0" : hata2.Text);
        }


        private void sps1_TextChanged(object sender, TextChangedEventArgs e)
        {
            sensor1SPS = Convert.ToDouble(sps1.Text == "" ? "0" : sps1.Text);
        }

        private void sps2_TextChanged(object sender, TextChangedEventArgs e)
        {
            sensor2SPS = Convert.ToDouble(sps2.Text == "" ? "0" : sps2.Text);
        }

        
    }
}
