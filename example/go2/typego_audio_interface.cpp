#include <crow.h>
#include <fstream>
#include <filesystem>
#include <alsa/asoundlib.h>
#include <iostream>

enum class ExecutionStatus {
    SUCCESS,
    TIMEOUT,
    ERROR
};

using ExecutionResult = std::pair<ExecutionStatus, std::string>;

bool playWav(const char* filename, int card = 5, int device = 0);

int main(int argc, char **argv) {
    // Ensure upload directory exists
    std::string upload_dir = "/tmp/audio_uploads";
    try {
        std::filesystem::create_directories(upload_dir);
    } catch (const std::exception& e) {
    }

    crow::SimpleApp app;
    CROW_ROUTE(app, "/")([](){
        return "Go2 audio interface is ready!";
    });

    // File upload route - receives WAV file and plays it
    CROW_ROUTE(app, "/upload_play").methods(crow::HTTPMethod::POST)([](const crow::request& req) {
        crow::multipart::message msg(req);
        
        // Look for the audio file in the multipart data
        auto audio_part = msg.get_part_by_name("audio");
        if (audio_part.body.empty()) {
            return crow::response(400, crow::json::wvalue{
                {"status", static_cast<int>(ExecutionStatus::ERROR)},
                {"message", "No audio file provided. Use 'audio' as the form field name."}
            });
        }
        
        // Get filename from Content-Disposition header
        std::string filename = "uploaded_audio.wav";
        auto header_obj = audio_part.get_header_object("Content-Disposition");
        auto it = header_obj.params.find("filename");
        if (it != header_obj.params.end()) {
            filename = it->second;
        }
        
        // Validate WAV extension
        if (filename.size() < 4 || filename.substr(filename.size() - 4) != ".wav") {
            return crow::response(400, crow::json::wvalue{
                {"status", static_cast<int>(ExecutionStatus::ERROR)},
                {"message", "Only WAV files are supported"}
            });
        }
        
        // Save the uploaded file
        std::string filepath = "/tmp/audio_uploads/" + filename;
        std::ofstream outfile(filepath, std::ios::binary);
        if (!outfile) {
            return crow::response(500, crow::json::wvalue{
                {"status", static_cast<int>(ExecutionStatus::ERROR)},
                {"message", "Failed to save uploaded file"}
            });
        }
        
        outfile.write(audio_part.body.data(), audio_part.body.size());
        outfile.close();
        
        // Play the uploaded WAV file
        bool ret = playWav(filepath.c_str(), 5, 0);
        
        if (ret) {
            return crow::response(200, crow::json::wvalue{
                {"status", static_cast<int>(ExecutionStatus::SUCCESS)},
                {"message", "Audio uploaded and played successfully!"},
                {"filename", filename},
                {"filepath", filepath}
            });
        } else {
            return crow::response(500, crow::json::wvalue{
                {"status", static_cast<int>(ExecutionStatus::ERROR)},
                {"message", "Audio uploaded but playback failed!"},
                {"filename", filename}
            });
        }
    });

    app.port(18081).multithreaded().run();
    
    return 0;
}


bool playWav(const char* filename, int card, int device) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return false;
    }
    
    // Read WAV header
    char header[44];
    file.read(header, 44);
    
    // Parse WAV header
    uint16_t audioFormat = *(uint16_t*)(header + 20);  // Should be 1 for PCM
    uint numChannels = *(uint16_t*)(header + 22);
    uint32_t sampleRate = *(uint32_t*)(header + 24);
    uint32_t byteRate = *(uint32_t*)(header + 28);
    uint16_t blockAlign = *(uint16_t*)(header + 32);
    uint16_t bitsPerSample = *(uint16_t*)(header + 34);
    
    // Verify it's a PCM WAV file
    if (audioFormat != 1) {
        std::cerr << "Not a PCM WAV file" << std::endl;
        return false;
    }
    
    std::cout << "WAV file info:" << std::endl;
    std::cout << "  Channels: " << numChannels << std::endl;
    std::cout << "  Sample rate: " << sampleRate << " Hz" << std::endl;
    std::cout << "  Bits per sample: " << bitsPerSample << std::endl;
    std::cout << "  Byte rate: " << byteRate << " bytes/sec" << std::endl;
    
    // Open PCM device
    snd_pcm_t *pcm_handle;
    char pcm_name[32];
    sprintf(pcm_name, "plughw:%d,%d", card, device);
    
    if (snd_pcm_open(&pcm_handle, pcm_name, SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        std::cerr << "Cannot open audio device: " << pcm_name << std::endl;
        return false;
    }
    
    // Configure PCM with WAV file parameters
    snd_pcm_hw_params_t *params;
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(pcm_handle, params);
    
    // Set parameters from WAV file
    snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    
    // Set format based on bits per sample
    snd_pcm_format_t format;
    if (bitsPerSample == 8) {
        format = SND_PCM_FORMAT_U8;
    } else if (bitsPerSample == 16) {
        format = SND_PCM_FORMAT_S16_LE;
    } else if (bitsPerSample == 24) {
        format = SND_PCM_FORMAT_S24_LE;
    } else if (bitsPerSample == 32) {
        format = SND_PCM_FORMAT_S32_LE;
    } else {
        std::cerr << "Unsupported bit depth: " << bitsPerSample << std::endl;
        snd_pcm_close(pcm_handle);
        return false;
    }
    
    snd_pcm_hw_params_set_format(pcm_handle, params, format);
    snd_pcm_hw_params_set_channels(pcm_handle, params, numChannels);
    
    unsigned int rate = sampleRate;
    int dir = 0;
    snd_pcm_hw_params_set_rate_near(pcm_handle, params, &rate, &dir);
    
    if (rate != sampleRate) {
        std::cerr << "Warning: Requested rate " << sampleRate 
                  << "Hz, got " << rate << "Hz" << std::endl;
    }
    
    // Apply parameters
    if (snd_pcm_hw_params(pcm_handle, params) < 0) {
        std::cerr << "Cannot set parameters" << std::endl;
        snd_pcm_close(pcm_handle);
        return false;
    }
    
    // Get actual parameters
    snd_pcm_hw_params_get_channels(params, &numChannels);
    snd_pcm_hw_params_get_rate(params, &rate, &dir);
    
    std::cout << "ALSA configured:" << std::endl;
    std::cout << "  Channels: " << numChannels << std::endl;
    std::cout << "  Sample rate: " << rate << " Hz" << std::endl;
    
    // Prepare PCM
    if (snd_pcm_prepare(pcm_handle) < 0) {
        std::cerr << "Cannot prepare PCM" << std::endl;
        snd_pcm_close(pcm_handle);
        return false;
    }
    
    // Play audio
    const int BUFFER_SIZE = 4096;
    char buffer[BUFFER_SIZE];
    snd_pcm_uframes_t frames;
    
    while (!file.eof()) {
        file.read(buffer, BUFFER_SIZE);
        size_t bytes_read = file.gcount();
        
        if (bytes_read == 0) break;
        
        // Calculate number of frames (1 frame = samples for all channels)
        frames = bytes_read / (numChannels * (bitsPerSample / 8));
        
        ssize_t result = snd_pcm_writei(pcm_handle, buffer, frames);
        
        if (result == -EPIPE) {
            std::cerr << "Underrun occurred" << std::endl;
            snd_pcm_prepare(pcm_handle);
        } else if (result < 0) {
            std::cerr << "Error writing to PCM device: " << snd_strerror(result) << std::endl;
            break;
        } else if (result != (ssize_t)frames) {
            std::cerr << "Short write (expected " << frames 
                      << ", wrote " << result << ")" << std::endl;
        }
    }
    
    // Wait for playback to complete
    snd_pcm_drain(pcm_handle);
    snd_pcm_close(pcm_handle);
    file.close();
    
    return true;
}