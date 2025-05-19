using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;
using TestMqttBroker.Services;

namespace TestMqttBroker;

public class Program
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
                services.AddSingleton<IMqttBrokerService, MqttBrokerService>();
            });

        using var host = builder.Build();

        var mqttBroker = host.Services.GetRequiredService<IMqttBrokerService>();
        await mqttBroker.StartAsync();

        // Keep the application running
        await host.RunAsync();
    }
}
