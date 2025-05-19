using System;
using System.Linq;
using System.Numerics;
using System.Text.Json;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;
using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Formatter;
using MQTTnet.Protocol;

namespace ZedConsoleApp.Services
{
    public class MqttClientService(ILogger<MqttClientService> logger)
    {
        private IMqttClient _mqttClient;

        public async Task InitializeMqttClient()
        {
            try
            {
                var mqttFactory = new MqttFactory();
                _mqttClient = mqttFactory.CreateMqttClient();

                var options = new MqttClientOptionsBuilder()
                    .WithTcpServer("localhost", 1883) // Replace with your MQTT broker address
                    .WithProtocolVersion(MqttProtocolVersion.V500)
                    .WithClientId($"ZedConsoleApp_{Guid.NewGuid()}")
                    .Build();

                await _mqttClient.ConnectAsync(options);

                logger.LogInformation("MQTT client connected successfully.");
            }
            catch (Exception ex)
            {
                logger.LogError(ex, "Failed to initialize MQTT client.");
            }
        }

        public async Task PublishPosition(Vector3 position)
        {
            if (_mqttClient?.IsConnected != true)
            {
                var error = new Exception("Cannot publish: Bluetooth or MQTT is not connected.");
                logger.LogError(error, "MQTT client is not connected.");
                throw error;
            }

            var data = new PublishData
            {
                x = position.X,
                y = position.Y,
                z = position.Z
            };

            var json = JsonSerializer.Serialize(data);
            var message = new MqttApplicationMessageBuilder()
                .WithTopic("objectPosition")
                .WithPayload(json)
                .WithQualityOfServiceLevel(MqttQualityOfServiceLevel.ExactlyOnce)
                .Build();

            await _mqttClient.PublishAsync(message);
        }

        private class PublishData
        {
            public float x { get; set; }
            public float y { get; set; }
            public float z { get; set; }
        }
    }
}