# Real-Time Wind Noise and Reverberation Suppression for Cochlear Implants

A real-time embedded speech enhancement system designed to improve speech intelligibility for cochlear implant users by suppressing wind noise and reverberation. The project combines Digital Signal Processing (DSP) and lightweight Machine Learning techniques and is targeted for deployment on the STM32H723ZG microcontroller.

---

## 📌 Overview

Cochlear implant users often face difficulty understanding speech in environments affected by wind noise and reverberation. This project implements a hybrid DSP and neural network–based solution that performs real-time speech enhancement directly on embedded hardware with low latency and high efficiency.

---

## ✨ Features

- Real-time wind noise suppression
- Reverberation reduction
- Hybrid DSP + Machine Learning approach
- STFT/FFT-based spectral processing
- Spectral subtraction with overlap-add reconstruction
- Tiny-CRN / RNNoise integration
- I2S + DMA-based continuous audio streaming
- Low-latency embedded implementation (< 25 ms)

---

## 🛠️ Hardware Used

- STM32H723ZG Nucleo-144
- INMP441 MEMS Microphone
- I2S DAC / Audio Output Module
- Push Button for bypass control
- Status LED

---

## 💻 Software & Tools

- Python
- TensorFlow / Keras
- NumPy, SciPy, Librosa
- Matplotlib
- STM32CubeIDE
- CMSIS-DSP
- CMSIS-NN

---

## ⚙️ System Workflow

1. Audio acquisition using INMP441
2. Frame segmentation and windowing
3. STFT / FFT
4. Noise estimation
5. Spectral subtraction
6. Neural network inference
7. iFFT and overlap-add reconstruction
8. Real-time audio playback

---

## 📊 Results

- Significant reduction in wind noise and reverberation
- Improved speech intelligibility
- Real-time processing with minimal latency
- Efficient implementation on embedded hardware

---

## 🎯 Applications

- Cochlear Implants
- Hearing Aids
- Assistive Listening Devices
- Smart Earphones
- Embedded Audio Processing Systems

---

## 🚀 Future Scope

- ASIC/SoC integration for implantable devices
- Multi-microphone beamforming
- Personalized hearing profiles
- Model optimization through pruning and quantization

---

## 📂 Repository Structure

```text
.
├── python_simulation/
├── stm32_firmware/
├── hardware/
├── results/
├── docs/
├── README.md
└── requirements.txt
