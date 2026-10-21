from __future__ import annotations

import torch
from torch import Tensor, nn


class CausalResidualBlock(nn.Module):
    def __init__(self, channels: int, kernel_size: int, dilation: int, dropout: float) -> None:
        super().__init__()
        self.left_padding = dilation * (kernel_size - 1)
        self.depthwise = nn.Conv1d(
            channels,
            channels,
            kernel_size=kernel_size,
            dilation=dilation,
            groups=channels,
        )
        self.pointwise = nn.Conv1d(channels, channels, kernel_size=1)
        self.norm = nn.LayerNorm(channels)
        self.dropout = nn.Dropout(dropout)

    def forward(self, x: Tensor) -> Tensor:
        # x is batch x time x channel. Only the left side is padded.
        y = x.transpose(1, 2)
        y = torch.nn.functional.pad(y, (self.left_padding, 0))
        y = self.pointwise(self.depthwise(y)).transpose(1, 2)
        y = self.dropout(torch.nn.functional.gelu(self.norm(y)))
        return x + y


class CymbalTCN(nn.Module):
    def __init__(
        self,
        input_dim: int = 576,
        hidden_dim: int = 64,
        dilations: tuple[int, ...] = (1, 2, 4, 8),
        kernel_size: int = 3,
        dropout: float = 0.1,
    ) -> None:
        super().__init__()
        self.input_norm = nn.LayerNorm(input_dim)
        self.projection = nn.Linear(input_dim, hidden_dim)
        self.blocks = nn.ModuleList(
            [CausalResidualBlock(hidden_dim, kernel_size, dilation, dropout) for dilation in dilations]
        )
        self.output_norm = nn.LayerNorm(hidden_dim)
        self.classifier = nn.Linear(hidden_dim, 2)
        self.model_args = {
            "input_dim": input_dim,
            "hidden_dim": hidden_dim,
            "dilations": list(dilations),
            "kernel_size": kernel_size,
            "dropout": dropout,
        }

    def forward(self, features: Tensor) -> Tensor:
        x = torch.nn.functional.gelu(self.projection(self.input_norm(features)))
        for block in self.blocks:
            x = block(x)
        return self.classifier(self.output_norm(x))


def make_model(config: dict) -> CymbalTCN:
    model = config["model"]
    return CymbalTCN(
        input_dim=int(model["input_dim"]),
        hidden_dim=int(model["hidden_dim"]),
        dilations=tuple(int(value) for value in model["dilations"]),
        kernel_size=int(model["kernel_size"]),
        dropout=float(model["dropout"]),
    )

