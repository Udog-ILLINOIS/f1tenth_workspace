"""Raceline prediction network (paper §V-A, Fig. 3).

Structure [PAPER]: two dilated-TCN encoders (two residual 1D-conv blocks with BatchNorm each), the future
encoding avg-pooled from xW to W, conv fusion and multi-head temporal attention in parallel, flatten both plus
the target window, concatenate, two-layer MLP -> T offsets. Sizes are [GAP] (config['model']).
"""
import torch
from torch import nn


class ResidualBlock(nn.Module):
    """[GAP] non-causal ('same' padding) dilated convs: the whole window is known geometry, not a time series."""

    def __init__(self, c_in, c_out, kernel, dilation, dropout):
        super().__init__()
        pad = dilation * (kernel - 1) // 2
        self.body = nn.Sequential(
            nn.Conv1d(c_in, c_out, kernel, padding=pad, dilation=dilation), nn.BatchNorm1d(c_out), nn.ReLU(),
            nn.Dropout(dropout),
            nn.Conv1d(c_out, c_out, kernel, padding=pad, dilation=dilation), nn.BatchNorm1d(c_out))
        self.skip = nn.Conv1d(c_in, c_out, 1) if c_in != c_out else nn.Identity()
        self.act = nn.ReLU()

    def forward(self, x):
        return self.act(self.body(x) + self.skip(x))


class TCN(nn.Module):
    def __init__(self, c_in, c, kernel, dilations, dropout):
        super().__init__()
        chans = [c_in] + [c] * len(dilations)
        self.blocks = nn.Sequential(*[ResidualBlock(chans[i], chans[i + 1], kernel, dl, dropout)
                                      for i, dl in enumerate(dilations)])

    def forward(self, x):
        return self.blocks(x)


class RacelineNet(nn.Module):
    def __init__(self, history_W, future_x, target_T, channels, kernel, dilations, attn_heads, mlp_hidden,
                 dropout, **_):
        super().__init__()
        W, C, T = history_W, channels, target_T
        self.hist_enc = TCN(4, C, kernel, dilations, dropout)            # B x 4 x W   -> B x C x W
        self.fut_enc = TCN(3, C, kernel, dilations, dropout)             # B x 3 x xW  -> B x C x xW
        self.pool = nn.AvgPool1d(future_x)                               # xW -> W
        self.conv_fusion = nn.Sequential(nn.Conv1d(2 * C, C, kernel, padding=kernel // 2), nn.ReLU())
        # [GAP] queries from the future encoding, keys/values from the history encoding.
        self.attn = nn.MultiheadAttention(C, attn_heads, dropout=dropout, batch_first=True)
        self.mlp = nn.Sequential(nn.Linear(2 * C * W + 3 * T, mlp_hidden), nn.ReLU(), nn.Dropout(dropout),
                                 nn.Linear(mlp_hidden, T))

    def forward(self, hist, fut, tgt):
        h = self.hist_enc(hist)
        f = self.pool(self.fut_enc(fut))
        fused = self.conv_fusion(torch.cat([h, f], dim=1))
        a, _ = self.attn(f.transpose(1, 2), h.transpose(1, 2), h.transpose(1, 2))
        z = torch.cat([fused.flatten(1), a.flatten(1), tgt.flatten(1)], dim=1)
        return self.mlp(z)
