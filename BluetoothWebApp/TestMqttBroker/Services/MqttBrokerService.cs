using Microsoft.Extensions.Logging;
using MQTTnet;
using MQTTnet.Server;

namespace TestMqttBroker.Services;

public interface IMqttBrokerService
{
    Task StartAsync();
    Task StopAsync();
}

public class MqttBrokerService : IMqttBrokerService
{
    private readonly ILogger<MqttBrokerService> _logger;
    private MqttServer? _mqttServer;

    public MqttBrokerService(ILogger<MqttBrokerService> logger)
    {
        _logger = logger;
    }

    public async Task StartAsync()
    {
        try
        {
            var mqttFactory = new MqttFactory();
            var options = new MqttServerOptionsBuilder()
                .WithDefaultEndpoint()
                .WithDefaultEndpointPort(1883)
                .Build();

            _mqttServer = mqttFactory.CreateMqttServer(options);

        _mqttServer.ClientConnectedAsync += args =>
        {
            _logger.LogInformation("Client connected: {ClientId}", args.ClientId);
            return Task.CompletedTask;
        };

        _mqttServer.ClientDisconnectedAsync += args =>
        {
            _logger.LogInformation("Client disconnected: {ClientId}", args.ClientId);
            return Task.CompletedTask;
        };

        _mqttServer.InterceptingPublishAsync += args =>
        {
            _logger.LogInformation(
                "Message received - Topic: {Topic}, QoS: {QoS}, Payload: {Payload}",
                args.ApplicationMessage.Topic,
                args.ApplicationMessage.QualityOfServiceLevel,
                args.ApplicationMessage.ConvertPayloadToString());
            return Task.CompletedTask;
        };

            await _mqttServer.StartAsync();
            _logger.LogInformation("MQTT Broker started on port 1883");
        }
        catch (Exception ex) when (ex is System.Net.Sockets.SocketException socketEx && socketEx.ErrorCode == 48)
        {
            _logger.LogError("Port 1883 is already in use. Please ensure no other MQTT broker is running.");
            throw new InvalidOperationException("MQTT broker port is already in use. Please stop any existing broker first.", ex);
        }
    }

    public async Task StopAsync()
    {
        if (_mqttServer != null)
        {
            await _mqttServer.StopAsync();
            _logger.LogInformation("MQTT Broker stopped");
        }
    }
}
