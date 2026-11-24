# Tulio henrique Rozethal Marques - TCC VRM Trifásico - UNIFEI 
# Supervisorio via Python - Engenharia de Dados e Automação
# -*- coding: utf-8 -*-
import json
import threading
import time
import collections
from pathlib import Path

import pandas as pd
import plotly.graph_objects as go
import serial
import streamlit as st

PORTA = "COM5"
BAUD = 460800
BANDA = 0.02  # +-2%
MAX_PTS = 12000

CORES = {
    "i1": "#1f77b4",
    "i2": "#2ca02c",
    "i3": "#d98a1b"
}

CAUSAS = {
    0: "—",
    1: "fase aberta (corrente ≈ 0)",
    2: "sobrecorrente",
    3: "corrente reversa",
    4: "sobretensão"
}

CAB = [
    "t_ms", "rod", "st", "n", "v", "vr", "u",
    "i1", "i2", "i3", "d1", "d2", "d3",
    "inj", "act", "bal", "ovr", "ti", "td", "ca", "fk"
]

st.set_page_config(page_title="VRM Dashboard", layout="wide")


def metricas_captura(cap):
    pre = int(cap["pre"])
    vr = float(cap["vr"])

    v = [x / 1000.0 for x in cap["v"]]
    ii = {}
    for k in ("i1", "i2", "i3"):
        ii[k] = [x / 1000.0 for x in cap[k]]

    n = len(v)

    def media(seq, a, b):
        trecho = seq[max(a, 0):b]
        if not trecho:
            return float("nan")
        return sum(trecho) / len(trecho)

    ti = cap.get("ti", 0)
    td = cap.get("td", 0)
    if ti and td and td > ti:
        lat = (td - ti) / 1000.0
    else:
        lat = None

    v_pre = media(v, pre - 50, pre)
    if n > pre:
        v_min = min(v[pre:])
        v_max = max(v[pre:])
    else:
        v_min = float("nan")
        v_max = float("nan")

    recup = None
    janela = 20
    for j in range(pre, n - janela):
        dentro = True
        for x in v[j:j + janela]:
            if abs(x - vr) > BANDA * vr:
                dentro = False
                break
        if dentro:
            recup = j - pre
            break

    antes = {}
    depois = {}
    for k, s in ii.items():
        antes[k] = media(s, pre - 50, pre)
        depois[k] = media(s, n - 50, n)

    return {
        "latencia_ms": lat,
        "v_pre": v_pre,
        "v_min": v_min,
        "v_max": v_max,
        "queda_v": v_pre - v_min,
        "recup_ms": recup,
        "i_antes": antes,
        "i_depois": depois,
        "i_tot_antes": sum(antes.values()),
        "i_tot_depois": sum(depois.values()),
    }


@st.cache_resource
def iniciar():
    ser = serial.Serial()
    ser.port = PORTA
    ser.baudrate = BAUD
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()

    ctx = {
        "ser": ser,
        "lock": threading.Lock(),
        "hist": collections.deque(maxlen=MAX_PTS),
        "caps": [],
        "msgs": collections.deque(maxlen=30)
    }

    pasta = Path("logs_vrm")
    pasta.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    arq = open(pasta / f"telemetria_{stamp}.csv", "w", buffering=1, newline="")
    arq.write(",".join(CAB) + "\n")

    def leitor():
        while True:
            try:
                linha = ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                time.sleep(0.5)
                continue

            if not linha:
                continue
            if linha.startswith("#"):
                ctx["msgs"].append(linha)
                continue
            if not linha.startswith("{"):
                continue

            try:
                d = json.loads(linha)
            except json.JSONDecodeError:
                continue

            try:
                if d.get("cap"):
                    n_cap = len(ctx["caps"]) + 1
                    caminho_cap = pasta / f"captura_{stamp}_{n_cap}.csv"
                    with open(caminho_cap, "w", newline="") as f:
                        f.write("t_rel_ms,v_V,i1_A,i2_A,i3_A\n")
                        for j in range(len(d["v"])):
                            t_rel = j - d["pre"]
                            f.write(
                                f"{t_rel},{d['v'][j]/1000:.3f},"
                                f"{d['i1'][j]/1000:.3f},{d['i2'][j]/1000:.3f},{d['i3'][j]/1000:.3f}\n"
                            )
                    with ctx["lock"]:
                        ctx["caps"].append(d)
                else:
                    row = {
                        "t_ms": d["t"], "rod": d["rod"], "st": d["st"], "n": d["n"],
                        "v": d["v"], "vr": d["vr"], "u": d["u"],
                        "i1": d["i"][0], "i2": d["i"][1], "i3": d["i"][2],
                        "d1": d["d"][0], "d2": d["d"][1], "d3": d["d"][2],
                        "inj": d["inj"], "act": d["act"], "bal": d["bal"], "ovr": d["ovr"],
                        "ti": d["ti"], "td": d["td"], "ca": d["ca"], "fk": d["fk"]
                    }
                    arq.write(",".join(str(row[c]) for c in CAB) + "\n")
                    with ctx["lock"]:
                        ctx["hist"].append(row)
            except (KeyError, IndexError, TypeError):
                continue

    t = threading.Thread(target=leitor, daemon=True)
    t.start()
    return ctx


try:
    ctx = iniciar()
except Exception as e:
    st.error(f"Erro ao abrir {PORTA}: {e}. Feche o Serial Monitor da  IDE.")
    st.stop()


def enviar(cmd):
    try:
        ctx["ser"].write((cmd + "\n").encode())
    except Exception:
        pass


st.title("⚡ VRM Trifásico — Monitoramento e Teste de Tolerância a Falhas")
st.caption("TCC — Engenharia de Controle e Automação · UNIFEI")

with st.sidebar:
    st.header("Comandos")
    c1, c2 = st.columns(2)
    c1.button("▶ Ligar", on_click=enviar, args=("G",))
    c2.button("■ Desligar", on_click=enviar, args=("X",))

    st.subheader("Injetar falha")
    f1, f2, f3 = st.columns(3)
    f1.button("Fase 1", on_click=enviar, args=("F1",))
    f2.button("Fase 2", on_click=enviar, args=("F2",))
    f3.button("Fase 3", on_click=enviar, args=("F3",))

    st.button("↺ Restaurar as 3 fases", on_click=enviar, args=("R",))
    st.button("⚖️ Calibrar zero ACS712", on_click=enviar, args=("C",))

    def _bal():
        if st.session_state["bal_ui"]:
            enviar("B1")
        else:
            enviar("B0")

    def _vref():
        enviar(f"V{st.session_state['vref_ui']:.2f}")

    st.toggle("Balanceamento de corrente", value=True, key="bal_ui", on_change=_bal)
    st.number_input("Vref (V)", 2.0, 5.5, 5.0, 0.1, key="vref_ui", on_change=_vref)
    janela = st.slider("Janela dos gráficos (s)", 5, 120, 30)

    with st.expander("Roteiro do teste"):
        st.markdown(
            "1. Com a ATX ligada e o VRM parado, clique em **⚖️ Calibrar zero ACS712** e confira se as correntes zeraram.\n"
            "2. Clique em **▶ Ligar** e espere Vout estabilizar em ~5 V.\n"
            "3. Carga entre 1,5 A e 4 A no total (2 resistores de 2,7 Ω em paralelo ≈ 3,7 A).\n"
            "4. **Injetar falha** em uma fase.\n"
            "5. Ver o painel *Transitório da falha* (captura a 1 kHz).\n"
            "6. **Restaurar** e repetir com o balanceamento ligado/desligado.\n\n"
            "Os CSVs ficam em `logs_vrm/` para comparar com o MATLAB."
        )

    with st.expander("Mensagens do ESP32"):
        for m in list(ctx["msgs"])[-8:]:
            st.text(m)


def cartao_fase(col, k, row):
    bit = 1 << (k - 1)
    inj = row["inj"]
    act = row["act"]

    if not row["rod"]:
        col.info(f"Fase {k}: parada")
    elif (inj & bit) and (act & bit):
        col.warning(f"Fase {k}: FALHA injetada, ainda não detectada")
    elif not (act & bit):
        col.error(f"Fase {k}: ISOLADA")
    else:
        col.success(f"Fase {k}: OK · {row[f'i{k}']:.2f} A")


@st.fragment(run_every=0.25)
def painel(janela):
    with ctx["lock"]:
        rows = list(ctx["hist"])
        caps = list(ctx["caps"])

    if not rows:
        st.info("Aguardando telemetria do ESP32…")
        return

    df = pd.DataFrame(rows)
    df["t"] = df["t_ms"] / 1000.0
    ult = rows[-1]
    t_fim = df["t"].iloc[-1]
    dfj = df[df["t"] >= t_fim - janela]

    # status das fases
    cols = st.columns(3)
    for k in (1, 2, 3):
        cartao_fase(cols[k - 1], k, ult)

    if ult["st"] == 2:
        causa = CAUSAS.get(ult["ca"], "?")
        defasagem = 360 // max(ult["n"], 1)
        st.error(
            f"MODO DEGRADADO — {ult['n']}/3 fases. Causa: {causa}. "
            f"Defasagem reajustada para {defasagem}°."
        )

    k1, k2, k3, k4, k5, k6 = st.columns(6)
    k1.metric("Vout", f"{ult['v']:.3f} V", f"{ult['v'] - ult['vr']:+.3f} V vs Vref")
    k2.metric("Fases ativas", f"{ult['n']}/3")
    k3.metric("Duty (PI)", f"{ult['u']:.1f} %")
    k4.metric("Corrente total", f"{ult['i1'] + ult['i2'] + ult['i3']:.2f} A")
    k5.metric("Vref efetivo", f"{ult['vr']:.2f} V")
    k6.metric(
        "Ticks perdidos",
        int(ult["ovr"]),
        help="Se crescer, o laço de 10 kHz não está cabendo em 100 µs."
    )

    vlines = []
    if ult["ti"]:
        vlines.append((ult["ti"] / 1e6, "falha injetada", "red"))
    if ult["td"]:
        vlines.append((ult["td"] / 1e6, "detectada/isolada", "green"))

    def marca(fig):
        for x, txt, cor in vlines:
            if x >= t_fim - janela:
                fig.add_vline(x=x, line_dash="dash", line_color=cor, annotation_text=txt)
        return fig

    g1, g2 = st.columns(2)

    fig_v = go.Figure()
    fig_v.add_trace(go.Scatter(x=dfj["t"], y=dfj["v"], name="Vout", line=dict(color="#c0392b")))
    vr = ult["vr"]
    fig_v.add_hrect(y0=vr * (1 - BANDA), y1=vr * (1 + BANDA), fillcolor="green", opacity=0.08, line_width=0)
    fig_v.update_layout(
        title="Tensão de saída (faixa ±2 %)",
        height=300,
        margin=dict(l=10, r=10, t=40, b=10),
        xaxis_title="s",
        yaxis_title="V"
    )
    g1.plotly_chart(marca(fig_v), key="g_vout")

    fig_i = go.Figure()
    for k in ("i1", "i2", "i3"):
        fig_i.add_trace(go.Scatter(x=dfj["t"], y=dfj[k], name=f"Fase {k[1]}", line=dict(color=CORES[k])))
    fig_i.update_layout(
        title="Corrente por fase (ACS712)",
        height=300,
        margin=dict(l=10, r=10, t=40, b=10),
        xaxis_title="s",
        yaxis_title="A"
    )
    g2.plotly_chart(marca(fig_i), key="g_i")

    g3, g4 = st.columns(2)

    fig_d = go.Figure()
    for k, c in (("d1", CORES["i1"]), ("d2", CORES["i2"]), ("d3", CORES["i3"])):
        fig_d.add_trace(go.Scatter(x=dfj["t"], y=dfj[k], name=f"Fase {k[1]}", line=dict(color=c)))
    fig_d.update_layout(
        title="Duty por fase (PI + balanceamento)",
        height=300,
        margin=dict(l=10, r=10, t=40, b=10),
        xaxis_title="s",
        yaxis_title="%"
    )
    g3.plotly_chart(marca(fig_d), key="g_d")

    fig_n = go.Figure()
    fig_n.add_trace(go.Scatter(x=dfj["t"], y=dfj["n"], name="Fases ativas", line=dict(shape="hv", color="#555")))
    fig_n.update_layout(
        title="Fases ativas (gain scheduling)",
        height=300,
        margin=dict(l=10, r=10, t=40, b=10),
        xaxis_title="s",
        yaxis=dict(range=[0, 3.5], dtick=1)
    )
    g4.plotly_chart(marca(fig_n), key="g_n")

    # captura do transitorio
    st.subheader("Transitório da falha (captura a 1 kHz)")
    if not caps:
        st.caption("Injete uma falha para gerar a captura (100 ms antes, 300 ms depois).")
        return

    cap = caps[-1]
    m = metricas_captura(cap)
    pre = cap["pre"]
    x = [j - pre for j in range(len(cap["v"]))]

    a, b, c, d = st.columns(4)
    lat_txt = f"{m['latencia_ms']:.2f} ms" if m["latencia_ms"] is not None else "—"
    rec_txt = f"{m['recup_ms']} ms" if m["recup_ms"] is not None else "não recuperou"

    a.metric("Latência de detecção", lat_txt)
    b.metric("Afundamento de Vout", f"{m['queda_v']*1000:.0f} mV", f"mín {m['v_min']:.3f} V")
    c.metric("Tempo de recuperação (±2 %)", rec_txt)
    d.metric("Corrente total antes → depois", f"{m['i_tot_antes']:.2f} → {m['i_tot_depois']:.2f} A")

    t1, t2 = st.columns(2)

    fig_cv = go.Figure()
    fig_cv.add_trace(go.Scatter(x=x, y=[q / 1000 for q in cap["v"]], name="Vout", line=dict(color="#c0392b")))
    fig_cv.add_vline(x=0, line_dash="dash", line_color="red", annotation_text="falha")
    if m["latencia_ms"] is not None:
        fig_cv.add_vline(x=m["latencia_ms"], line_dash="dash", line_color="green", annotation_text="detecção")
    fig_cv.update_layout(
        title=f"Vout durante a falha da fase {cap['fk']}",
        height=320,
        margin=dict(l=10, r=10, t=40, b=10),
        xaxis_title="ms",
        yaxis_title="V"
    )
    t1.plotly_chart(fig_cv, key="c_v")

    fig_ci = go.Figure()
    for k in ("i1", "i2", "i3"):
        fig_ci.add_trace(go.Scatter(x=x, y=[q / 1000 for q in cap[k]], name=f"Fase {k[1]}", line=dict(color=CORES[k])))
    fig_ci.add_vline(x=0, line_dash="dash", line_color="red")
    fig_ci.update_layout(
        title="Redistribuição de corrente",
        height=320,
        margin=dict(l=10, r=10, t=40, b=10),
        xaxis_title="ms",
        yaxis_title="A"
    )
    t2.plotly_chart(fig_ci, key="c_i")

    antes_lista = [round(m["i_antes"][k], 3) for k in ("i1", "i2", "i3")]
    depois_lista = [round(m["i_depois"][k], 3) for k in ("i1", "i2", "i3")]
    var_lista = []
    for a_, d_ in zip(antes_lista, depois_lista):
        if a_ and abs(a_) > 0.05:
            var_lista.append(f"{(d_ - a_) / a_ * 100:+.0f} %")
        else:
            var_lista.append("—")

    tab = pd.DataFrame({
        "Fase": ["1", "2", "3"],
        "Antes (A)": antes_lista,
        "Depois (A)": depois_lista,
        "Variação": var_lista,
    })
    st.dataframe(tab, hide_index=True)


painel(janela)