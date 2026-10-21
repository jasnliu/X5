from __future__ import annotations

import torch
from torch import Tensor, nn
import torchaudio


class LogMelFrontend(nn.Module):
    """Exact evaluation frontend used by the pinned EfficientSED fmn checkpoint."""

    sample_rate = 16_000
    hop_samples = 160
    hop_seconds = 0.010

    def __init__(self) -> None:
        super().__init__()
        self.register_buffer("window", torch.hann_window(400, periodic=False), persistent=False)
        self.register_buffer(
            "preemphasis", torch.tensor([[[-0.97, 1.0]]], dtype=torch.float32), persistent=False
        )
        mel, _ = torchaudio.compliance.kaldi.get_mel_banks(
            128,
            512,
            self.sample_rate,
            0.0,
            7000.0,
            vtln_low=100.0,
            vtln_high=-500.0,
            vtln_warp_factor=1.0,
        )
        mel = torch.nn.functional.pad(mel, (0, 1))
        self.register_buffer("mel_basis", mel, persistent=True)

    def forward(self, waveform: Tensor) -> Tensor:
        if waveform.ndim == 1:
            waveform = waveform.unsqueeze(0)
        waveform = torch.nn.functional.conv1d(waveform.unsqueeze(1), self.preemphasis).squeeze(1)
        stft = torch.stft(
            waveform,
            n_fft=512,
            hop_length=160,
            win_length=400,
            center=True,
            normalized=False,
            window=self.window,
            return_complex=True,
        )
        power = stft.abs().square()
        mel = torch.matmul(self.mel_basis, power)
        logmel = torch.log(torch.clamp(mel, min=1e-7))
        logmel = (logmel + 4.5) / 5.0
        return logmel.unsqueeze(1)

