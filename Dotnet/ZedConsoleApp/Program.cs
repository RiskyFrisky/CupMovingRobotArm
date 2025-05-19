using System.Threading.Tasks;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;
using ZedConsoleApp.Services;

namespace ZedConsoleApp
{
    internal class Program
    {
        public static async Task Main(string[] args)
        {
            var builder = Host.CreateDefaultBuilder(args)
                .ConfigureLogging(logging =>
                {
                    logging.ClearProviders();
                    logging.AddConsole();
                })
                .ConfigureServices((hostContext, services) =>
                {
                    services.AddSingleton<MqttClientService>();
                    services.AddSingleton<ZedService>();
                });

            using var host = builder.Build();

            var mqttClient = host.Services.GetRequiredService<MqttClientService>();
            var zedService = host.Services.GetRequiredService<ZedService>();
            
            await mqttClient.InitializeMqttClient();
            
            // Start the ZED service in a separate task
            var zedTask = Task.Run(() => zedService.Run());

            // Keep the application running
            await host.RunAsync();
        }
    }
}