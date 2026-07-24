#include "comm_host.h"
#include "foc_control.h"
#include "foc_config.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static float _parse_float(const char *s) { return strtof(s, NULL); }

static void _send_str(foc_comm_t *comm, const char *str)
{
    if (!comm->uart || !comm->uart->tx) return;
    while (*str) { comm->uart->tx(*str); str++; }
}

static int _split_args(const char *line, char *args[], int max_args)
{
    int count = 0;
    const char *p = line;
    while (*p && count < max_args)
    {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        args[count++] = (char *)p;
        while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') p++;
        if (*p) { *(char *)p = '\0'; p++; }
    }
    return count;
}

void comm_init(foc_comm_t *comm, foc_core_t *core, const hal_uart_t *uart)
{
    comm->uart = uart;
    comm->core = core;
    comm->rx_index = 0;
    comm->rx_ready = 0;
    memset(comm->rx_buf, 0, sizeof(comm->rx_buf));
    comm->monitor_enabled = 0;
    comm->monitor_count = 0;
    comm->monitor_interval_ms = 10;
    comm->last_monitor_ms = 0;
}

void comm_process_char(foc_comm_t *comm, char c)
{
    if (c == '\n' || c == '\r')
    {
        if (comm->rx_index > 0)
        {
            comm->rx_buf[comm->rx_index] = '\0';
            comm->rx_ready = 1;
        }
        comm->rx_index = 0;
    }
    else if (c == '\b' || c == 0x7F)
    {
        if (comm->rx_index > 0) comm->rx_index--;
    }
    else
    {
        if (comm->rx_index < sizeof(comm->rx_buf) - 1)
            comm->rx_buf[comm->rx_index++] = c;
    }
}

static void _cmd_pid(foc_comm_t *comm, char *args[], int n)
{
    if (!comm->core) return;
    if (n < 3) { _send_str(comm, "ERR: pid <loop> <kp> <ki>\n"); return; }
    int loop = atoi(args[0]);
    float kp = _parse_float(args[1]);
    float ki = _parse_float(args[2]);
    float kd = (n >= 4) ? _parse_float(args[3]) : 0.0f;
    foc_control_set_pid(&comm->core->ctrl, (uint32_t)loop, kp, ki, kd);

    char buf[64];
    snprintf(buf, sizeof(buf), "OK: pid%d kp=%.4f ki=%.4f\n", loop, kp, ki);
    _send_str(comm, buf);
}

static void _cmd_mode(foc_comm_t *comm, char *args[], int n)
{
    if (!comm->core) return;
    if (n < 1) { _send_str(comm, "ERR: mode <idle|pos|spd|cur>\n"); return; }

    if (strcmp(args[0], "idle") == 0)
        foc_control_set_mode(&comm->core->ctrl, FOC_MODE_IDLE);
    else if (strcmp(args[0], "pos") == 0)
        foc_control_set_mode(&comm->core->ctrl, FOC_MODE_POSITION);
    else if (strcmp(args[0], "spd") == 0)
        foc_control_set_mode(&comm->core->ctrl, FOC_MODE_SPEED);
    else if (strcmp(args[0], "cur") == 0)
        foc_control_set_mode(&comm->core->ctrl, FOC_MODE_TORQUE);
    else
    { _send_str(comm, "ERR: unknown mode\n"); return; }
    _send_str(comm, "OK\n");
}

static void _cmd_target(foc_comm_t *comm, char *args[], int n)
{
    if (!comm->core) return;
    if (n < 1) { _send_str(comm, "ERR: target <value>\n"); return; }

    float val = _parse_float(args[0]);
    foc_ctrl_mode_t mode = foc_control_get_mode(&comm->core->ctrl);

    switch (mode)
    {
    case FOC_MODE_POSITION: foc_control_set_target_pos(&comm->core->ctrl, val); break;
    case FOC_MODE_SPEED:    foc_control_set_target_spd(&comm->core->ctrl, val); break;
    case FOC_MODE_TORQUE:   foc_control_set_target_iq(&comm->core->ctrl, val); break;
    default:
        _send_str(comm, "ERR: no mode set\n"); return;
    }
    _send_str(comm, "OK\n");
}

static void _cmd_enable(foc_comm_t *comm)
{
    if (comm->core) foc_core_enable(comm->core);
    _send_str(comm, "OK: enabled\n");
}

static void _cmd_disable(foc_comm_t *comm)
{
    if (comm->core) foc_core_disable(comm->core);
    _send_str(comm, "OK: disabled\n");
}

static void _cmd_monitor(foc_comm_t *comm, char *args[], int n)
{
    if (n < 1)
    {
        comm->monitor_enabled = !comm->monitor_enabled;
        _send_str(comm, comm->monitor_enabled ? "OK: monitor on\n" : "OK: monitor off\n");
        return;
    }

    comm->monitor_count = 0;
    for (int i = 0; i < n && i < COMM_MAX_VARS; i++)
    {
        size_t len = strlen(args[i]);
        if (len >= sizeof(comm->monitor_copy[i]))
            len = sizeof(comm->monitor_copy[i]) - 1;
        memcpy(comm->monitor_copy[i], args[i], len);
        comm->monitor_copy[i][len] = '\0';
        comm->monitor_vars[i] = comm->monitor_copy[i];
        comm->monitor_count++;
    }
    comm->monitor_enabled = 1;
    _send_str(comm, "OK: monitoring ");
    char buf[8];
    for (uint32_t i = 0; i < comm->monitor_count; i++)
    {
        snprintf(buf, sizeof(buf), "%s ", comm->monitor_vars[i]);
        _send_str(comm, buf);
    }
    _send_str(comm, "\n");
}

static void _cmd_status(foc_comm_t *comm)
{
    if (!comm->core) return;

    char buf[128];
    snprintf(buf, sizeof(buf),
             "state=%d mode=%d fault=%d en=%d\n"
             " pos=%.3f spd=%.3f id=%.3f iq=%.3f\n"
             " vbus=%.1f\n",
             (int)foc_core_get_state(comm->core),
             (int)foc_control_get_mode(&comm->core->ctrl),
             (int)foc_core_get_fault(comm->core),
             foc_core_is_enabled(comm->core),
             foc_core_get_angle(comm->core),
             foc_core_get_speed(comm->core),
             foc_core_get_id(comm->core),
             foc_core_get_iq(comm->core),
             foc_core_get_vbus(comm->core));
    _send_str(comm, buf);
}

static void _cmd_encoder(foc_comm_t *comm)
{
    if (!comm->core || !comm->core->encoder_hal || !comm->core->encoder_hal->get_angle)
    { _send_str(comm, "ERR: no encoder\n"); return; }

    float rad = comm->core->encoder_hal->get_angle();
    char buf[64];
    snprintf(buf, sizeof(buf), "mech_angle=%.4f rad (%.1f deg)\n", rad, rad * 57.2958f);
    _send_str(comm, buf);
}

static void _cmd_calib(foc_comm_t *comm)
{
    _send_str(comm, "calib not implemented via uart\n");
}

static void _cmd_stop(foc_comm_t *comm)
{
    if (!comm->core) return;
    foc_control_set_target_spd(&comm->core->ctrl, 0.0f);
    foc_control_set_target_iq(&comm->core->ctrl, 0.0f);
    _send_str(comm, "OK: stopped\n");
}

static void _cmd_help(foc_comm_t *comm)
{
    _send_str(comm,
        "Commands:\n"
        "  pid <loop> <kp> <ki> [kd]  set PID\n"
        "  mode <idle|pos|spd|cur>    set control mode\n"
        "  target <value>             set target\n"
        "  enable/disable             motor on/off\n"
        "  monitor [var1 var2 ...]    toggle/start monitor\n"
        "  status                     show all status\n"
        "  encoder                    read raw encoder angle\n"
        "  calib                      calibrate current offset\n"
        "  stop                       emergency stop\n"
        "  help                       show this message\n"
    );
}

void comm_process_line(foc_comm_t *comm)
{
    if (!comm->rx_ready) return;
    comm->rx_ready = 0;

    char *line = comm->rx_buf;
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    char *args[8];
    int n = _split_args(line, args, 8);
    if (n < 1) return;

    if (strcmp(args[0], "pid") == 0)        _cmd_pid(comm, args + 1, n - 1);
    else if (strcmp(args[0], "mode") == 0)  _cmd_mode(comm, args + 1, n - 1);
    else if (strcmp(args[0], "target") == 0) _cmd_target(comm, args + 1, n - 1);
    else if (strcmp(args[0], "enable") == 0) _cmd_enable(comm);
    else if (strcmp(args[0], "disable") == 0) _cmd_disable(comm);
    else if (strcmp(args[0], "monitor") == 0) _cmd_monitor(comm, args + 1, n - 1);
    else if (strcmp(args[0], "status") == 0) _cmd_status(comm);
    else if (strcmp(args[0], "calib") == 0)  _cmd_calib(comm);
    else if (strcmp(args[0], "encoder") == 0) _cmd_encoder(comm);
    else if (strcmp(args[0], "stop") == 0)   _cmd_stop(comm);
    else if (strcmp(args[0], "help") == 0)   _cmd_help(comm);
    else
    {
        _send_str(comm, "ERR: unknown cmd '");
        _send_str(comm, args[0]);
        _send_str(comm, "'. type 'help'\n");
    }
}

static void _output_monitor(foc_comm_t *comm)
{
    if (!comm->monitor_enabled || !comm->core) return;

    char buf[128];
    char *p = buf;
    int rem = sizeof(buf);

    for (uint32_t i = 0; i < comm->monitor_count && rem > 0; i++)
    {
        const char *name = comm->monitor_vars[i];
        float val = 0.0f;
        if (strcmp(name, "pos") == 0)      val = foc_core_get_angle(comm->core);
        else if (strcmp(name, "spd") == 0) val = foc_core_get_speed(comm->core);
        else if (strcmp(name, "iq") == 0)  val = foc_core_get_iq(comm->core);
        else if (strcmp(name, "id") == 0)  val = foc_core_get_id(comm->core);
        else if (strcmp(name, "vbus") == 0)val = foc_core_get_vbus(comm->core);

        int n = snprintf(p, rem, "%s=%.4f ", name, val);
        p += n; rem -= n;
        if (rem <= 0) break;
    }

    if (buf[0])
    {
        _send_str(comm, "t=");
        char tbuf[16];
        snprintf(tbuf, sizeof(tbuf), "%.6f, ",
                 (comm->core->loop_count * FOC_DT_CURRENT));
        _send_str(comm, tbuf);
        _send_str(comm, buf);
        _send_str(comm, "\n");
    }
}

void comm_tick(foc_comm_t *comm)
{
    if (!comm || !comm->uart || !comm->uart->rx) return;

    char c;
    uint32_t poll_cnt = 0;
    while (poll_cnt < 8 && comm->uart->rx(&c))
    {
        if (comm->uart->tx) comm->uart->tx(c);
        comm_process_char(comm, c);
        if (comm->rx_ready) comm_process_line(comm);
        poll_cnt++;
    }

    _output_monitor(comm);
}
