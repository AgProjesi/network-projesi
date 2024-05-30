#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/hex-grid-position-allocator.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/lora-channel.h"
#include "ns3/lora-device-address-generator.h"
#include "ns3/lora-helper.h"
#include "ns3/lora-phy-helper.h"
#include "ns3/lorawan-mac-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/log.h"
#include "ns3/lora-helper.h"
#include "ns3/network-module.h"
#include "ns3/network-server-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rectangle.h"
#include "ns3/string.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/periodic-sender.h"
#include "ns3/position-allocator.h"
#include "ns3/simulator.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "time.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("OurNetworkProject");

long long sentTime; // Gönderim zamanı
long long receivedTime; // Alınan zaman
std::vector<int> receivedPackets; // Alınan paketlerin sayısı
std::vector<int> sentPackets; // Gönderilen paketlerin sayısı
std::vector<long long> delays; // Gecikmelerin listesi

auto packetsSent = std::vector<int>(6, 0); // Gönderilen paketlerin listesi
auto packetsReceived = std::vector<int>(6, 0); // Alınan paketlerin listesi

// Şu anki zamanı milisaniye cinsinden döndürür
long long getCurrentTime() {
    auto currentTime = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    return currentTime.time_since_epoch().count();
}

// Paket gönderimi olayı gerçekleştiğinde çağrılır
void OnTransmission(Ptr<const Packet> packet, uint32_t senderNodeId) {
    NS_LOG_FUNCTION(packet << senderNodeId);
    LoraTag tag;
    packet->PeekPacketTag(tag);
    packetsSent.at(tag.GetSpreadingFactor() - 7)++;
    sentPackets.insert(sentPackets.end(), packetsSent.begin(), packetsSent.end());
    sentTime = getCurrentTime();
    std::cout << "sentPackets : " << sentPackets.size()<< " time : " << sentTime << std::endl;
}

// Paket alımı olayı gerçekleştiğinde çağrılır
void OnPacketReception(Ptr<const Packet> packet, uint32_t receiverNodeId) {
    NS_LOG_FUNCTION(packet << receiverNodeId);
    LoraTag tag;
    packet->PeekPacketTag(tag);
    packetsReceived.at(tag.GetSpreadingFactor() - 7)++;
    receivedPackets.insert(receivedPackets.end(), packetsReceived.begin(), packetsReceived.end());
    receivedTime = getCurrentTime();
    delays.push_back(receivedTime - sentTime);
    std::cout << "receivedPackets : " << receivedPackets.size() << " time : " << receivedTime << std::endl;
}

int main(int argc, char* argv[]) {
    // Değişkenlerin tanımlanması ve varsayılan değerlerin atanması
    int appPeriodSeconds = 5; // Uygulama paketlerinin gönderim periyodu (saniye)
    int nDevices = 204; // NODE SAYISI, BURASI ÖNEMLİ BU SAYIYI DOĞRU BİR ŞEKİLDE 
    std::string outputFolder = "output"; // Çıktı klasörü

    // Komut satırı parametrelerinin işlenmesi
    CommandLine cmd;
    cmd.AddValue("nDevices", "Number of end devices", nDevices);
    cmd.AddValue("appPeriodSeconds", "Period of application packets", appPeriodSeconds);
    cmd.AddValue("OutputFolder", "Output folder", outputFolder);
    cmd.Parse(argc, argv);
    
    // NS-3 log bileşeninin tanımlanması
    NS_LOG_INFO("Creating the channel...");

    // Yayılma modelinin oluşturulması
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76); 
    loss->SetReference(1, 7.7);

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);

    LorawanMacHelper macHelper = LorawanMacHelper();

    LoraHelper helper = LoraHelper();
    helper.EnablePacketTracking(); 

    // Hareketlilik modeli için yardımcının ayarlanması
    MobilityHelper mobilityGw;
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
    allocator->Add(Vector(4444,4444,44));
    mobilityGw.SetPositionAllocator(allocator);
    mobilityGw.SetMobilityModel("ns3::ConstantPositionMobilityModel"); 

    // Gateway'lerin oluşturulması
    NS_LOG_INFO("Creating the gateway...");
    NodeContainer gateways;
    gateways.Create(1);
    mobilityGw.Install(gateways);
    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    // Son cihazların oluşturulması
    NS_LOG_INFO("Creating the end device...");
    std::string traceFile = "/home/serdar1/ns-allinone-3.41/ns-3.41/scratch/ns2mobility.tcl";
    Ns2MobilityHelper ns2 = Ns2MobilityHelper(traceFile);
    NodeContainer endDevices;
    endDevices.Create(nDevices); // end device sayısı
    ns2.Install();

    // Cihaz adreslerinin oluşturulması ve atanması
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen = CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);

    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    macHelper.SetAddressGenerator(addrGen);
    helper.Install(phyHelper, macHelper, endDevices);

    // Uygulama paketlerinin gönderilmesi için yardımcının ayarlanması ve kurulumu
    PeriodicSenderHelper appHelper = PeriodicSenderHelper();
    appHelper.SetPeriod(Seconds(appPeriodSeconds));
    ApplicationContainer appContainer = appHelper.Install(endDevices);

    // Paket alımı ve gönderimi olaylarının takip edilmesi
    for (auto node = gateways.Begin(); node != gateways.End(); node++)
    {
        (*node)->GetDevice(0)->GetObject<LoraNetDevice>()->GetPhy()->TraceConnectWithoutContext(
            "ReceivedPacket",
            MakeCallback(&OnPacketReception));
    }

    for (auto node = endDevices.Begin(); node != endDevices.End(); node++)
    {
        (*node)->GetDevice(0)->GetObject<LoraNetDevice>()->GetPhy()->TraceConnectWithoutContext(
            "StartSending",
            MakeCallback(&OnTransmission));
    }

    // Yayılma faktörlerinin ayarlanması
    std::vector<int> sfQuantity(6);
    sfQuantity = LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);

    // Simülasyonun çalıştırılması
    Simulator::Stop(Seconds(463));
    Simulator::Run();
    Simulator::Destroy();

    // Gecikme sürelerinin toplanması
    long long totalDelay = 0;
    for (auto it = delays.begin(); it != delays.end(); ++it) {
        totalDelay += *it;
    }

    // Başarı oranının ve ortalama gecikmenin hesaplanması
    float successRate = 100 * static_cast<float>(receivedPackets.size()) / sentPackets.size();
    float averageDelay = static_cast<float>(totalDelay) / delays.size();

    // Sonuçların dosyaya yazdırılması
    std::ofstream outputFile(outputFolder + "/sonuclar.txt");
    if (outputFile.is_open()) {
        outputFile << "Packet success rate: %" << std::fixed << std::setprecision(2) << successRate << std::endl;
        outputFile << "Average delay per packet: " << std::fixed << std::setprecision(2) << averageDelay << " milliseconds." << std::endl;
        outputFile.close();
        std::cout << "Sonuclar.txt dosyası başarıyla oluşturuldu." << std::endl;
    } else {
        std::cerr << "Hata: Çıktı dosyası açılamadı." << std::endl;
    }
    
    return 0;
}

