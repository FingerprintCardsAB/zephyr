/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <spi.h>
#include <clock_control.h>
#include <fsl_lpspi.h>

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SPI_LEVEL
#include <logging/sys_log.h>

#include "spi_context.h"

struct spi_mcux_config {
    LPSPI_Type *base;
    char *clock_name;
    clock_control_subsys_t clock_subsys;
    void (*irq_config_func)(struct device *dev);
};

struct spi_mcux_data {
    lpspi_master_handle_t m_handle;
    lpspi_slave_handle_t s_handle;
    struct spi_context ctx;
    size_t transfer_len;
};

static void spi_mcux_transfer_next_packet(struct device *dev)
{
    const struct spi_mcux_config *config = dev->config->config_info;
    struct spi_mcux_data *data = dev->driver_data;
    LPSPI_Type *base = config->base;
    struct spi_context *ctx = &data->ctx;
    lpspi_transfer_t transfer;
    status_t status;

    if ((ctx->tx_len == 0) && (ctx->rx_len == 0)) {
        /* nothing left to rx or tx, we're done! */
        spi_context_cs_control(&data->ctx, false);
        spi_context_complete(&data->ctx, 0);
        return;
    }

    transfer.configFlags = kLPSPI_MasterPcsContinuous |
                   (ctx->config->slave << LPSPI_MASTER_PCS_SHIFT);

    if (ctx->tx_len == 0) {
        /* rx only, nothing to tx */
        transfer.txData = NULL;
        transfer.rxData = ctx->rx_buf;
        transfer.dataSize = ctx->rx_len;
    } else if (ctx->rx_len == 0) {
        /* tx only, nothing to rx */
        transfer.txData = (u8_t *)ctx->tx_buf;
        transfer.rxData = NULL;
        transfer.dataSize = ctx->tx_len;
    } else if (ctx->tx_len == ctx->rx_len) {
        /* rx and tx are the same length */
        transfer.txData = (u8_t *) ctx->tx_buf;
        transfer.rxData = ctx->rx_buf;
        transfer.dataSize = ctx->tx_len;
    } else if (ctx->tx_len > ctx->rx_len) {
        /* Break up the tx into multiple transfers so we don't have to
         * rx into a longer intermediate buffer. Leave chip select
         * active between transfers.
         */
        transfer.txData = (u8_t *)ctx->tx_buf;
        transfer.rxData = ctx->rx_buf;
        transfer.dataSize = ctx->rx_len;
    } else {
        /* Break up the rx into multiple transfers so we don't have to
         * tx from a longer intermediate buffer. Leave chip select
         * active between transfers.
         */
        transfer.txData = (u8_t *)ctx->tx_buf;
        transfer.rxData = ctx->rx_buf;
        transfer.dataSize = ctx->tx_len;
    }

    data->transfer_len = transfer.dataSize;

    status = LPSPI_MasterTransferNonBlocking(base, &data->m_handle, &transfer);
    if (status != kStatus_Success) {
        SYS_LOG_ERR("Transfer could not start");
    }
}

static void spi_mcux_receive_next_packet(struct device *dev)
{
    const struct spi_mcux_config *config = dev->config->config_info;
    struct spi_mcux_data *data = dev->driver_data;
    LPSPI_Type *base = config->base;
    struct spi_context *ctx = &data->ctx;
    lpspi_transfer_t transfer;
    status_t status;

    if ((ctx->tx_len == 0) && (ctx->rx_len == 0)) {
        /* nothing left to rx or tx, we're done! */
        spi_context_cs_control(&data->ctx, false);
        spi_context_complete(&data->ctx, 0);
        return;
    }

    transfer.configFlags = (ctx->config->slave << LPSPI_SLAVE_PCS_SHIFT);

    if (ctx->tx_len == 0) {
        /* rx only, nothing to tx */
        transfer.txData = NULL;
        transfer.rxData = ctx->rx_buf;
        transfer.dataSize = ctx->rx_len;
    } else if (ctx->rx_len == 0) {
        /* tx only, nothing to rx */
        transfer.txData = (u8_t *) ctx->tx_buf;
        transfer.rxData = NULL;
        transfer.dataSize = ctx->tx_len;
    } else if (ctx->tx_len == ctx->rx_len) {
        /* rx and tx are the same length */
        transfer.txData = (u8_t *) ctx->tx_buf;
        transfer.rxData = ctx->rx_buf;
        transfer.dataSize = ctx->tx_len;
    } else if (ctx->tx_len > ctx->rx_len) {
        /* Break up the tx into multiple transfers so we don't have to
         * rx into a longer intermediate buffer. Leave chip select
         * active between transfers.
         */
        transfer.txData = (u8_t *) ctx->tx_buf;
        transfer.rxData = ctx->rx_buf;
        transfer.dataSize = ctx->rx_len;

    } else {
        /* Break up the rx into multiple transfers so we don't have to
         * tx from a longer intermediate buffer. Leave chip select
         * active between transfers.
         */
        transfer.txData = (u8_t *) ctx->tx_buf;
        transfer.rxData = ctx->rx_buf;
        transfer.dataSize = ctx->tx_len;
    }

    data->transfer_len = transfer.dataSize;

    status = LPSPI_SlaveTransferNonBlocking(base, &data->s_handle, &transfer);
    if (status != kStatus_Success) {
        SYS_LOG_ERR("Transfer could not start");
    }
}

static void spi_mcux_isr(void *arg)
{
    struct device *dev = (struct device *)arg;
    struct spi_mcux_data *data = dev->driver_data;
    const struct spi_mcux_config *config = dev->config->config_info;
    bool master;

    master = LPSPI_IsMaster(config->base);
    if (master) {
        LPSPI_MasterTransferHandleIRQ(config->base, &data->m_handle);
    } else {
        LPSPI_SlaveTransferHandleIRQ(config->base, &data->s_handle);
    }
}

static void spi_mcux_master_transfer_callback(LPSPI_Type *base,
        lpspi_master_handle_t *handle, status_t status, void *userData)
{
    struct device *dev = userData;
    struct spi_mcux_data *data = dev->driver_data;

    spi_context_update_tx(&data->ctx, 1, data->transfer_len);
    spi_context_update_rx(&data->ctx, 1, data->transfer_len);

    spi_mcux_transfer_next_packet(dev);
}

static void spi_mcux_slave_transfer_callback(LPSPI_Type *base,
        lpspi_slave_handle_t *handle, status_t status, void *userData)
{
    struct device *dev = userData;
    struct spi_mcux_data *data = dev->driver_data;

    spi_context_update_tx(&data->ctx, 1, data->transfer_len);
    spi_context_update_rx(&data->ctx, 1, data->transfer_len);

    spi_mcux_receive_next_packet(dev);
}

static int slave_init(LPSPI_Type *base, struct device *dev, const struct spi_config *spi_cfg)
{
    struct spi_mcux_data *data = dev->driver_data;
    lpspi_slave_config_t slave_config;
    u32_t word_size;

    LPSPI_SlaveGetDefaultConfig(&slave_config);

    if (spi_cfg->slave > 2) {
        SYS_LOG_ERR("Slave %d is greater than %d", spi_cfg->slave, 2);
        return -EINVAL;
    }

    word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
    if (word_size > 32) {
        SYS_LOG_ERR("Word size %d is greater than %d", word_size, 32);
        return -EINVAL;
    }

    slave_config.bitsPerFrame = word_size;

    slave_config.cpol =
        (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL)
        ? kLPSPI_ClockPolarityActiveLow
        : kLPSPI_ClockPolarityActiveHigh;

    slave_config.cpha =
        (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA)
        ? kLPSPI_ClockPhaseSecondEdge
        : kLPSPI_ClockPhaseFirstEdge;

    slave_config.direction =
        (spi_cfg->operation & SPI_TRANSFER_LSB)
        ? kLPSPI_LsbFirst
        : kLPSPI_MsbFirst;

    LPSPI_SlaveInit(base, &slave_config);

    LPSPI_SlaveTransferCreateHandle(base, &data->s_handle, spi_mcux_slave_transfer_callback, dev);

    data->ctx.config = spi_cfg;
    spi_context_cs_configure(&data->ctx);

    return 0;
}

static int master_init(LPSPI_Type *base, struct device *dev, const struct spi_config *spi_cfg)
{
    const struct spi_mcux_config *config = dev->config->config_info;
    struct spi_mcux_data *data = dev->driver_data;
    lpspi_master_config_t master_config;
    struct device *clock_dev;
    u32_t clock_freq;
    u32_t word_size;

    LPSPI_MasterGetDefaultConfig(&master_config);

    if (spi_cfg->slave > 2) {
        SYS_LOG_ERR("Slave %d is greater than %d", spi_cfg->slave, 2);
        return -EINVAL;
    }

    word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
    if (word_size > 32) {
        SYS_LOG_ERR("Word size %d is greater than %d", word_size, 32);
        return -EINVAL;
    }

    master_config.bitsPerFrame = word_size;

    master_config.cpol = (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL)
        ? kLPSPI_ClockPolarityActiveLow
        : kLPSPI_ClockPolarityActiveHigh;

    master_config.cpha = (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA)
        ? kLPSPI_ClockPhaseSecondEdge
        : kLPSPI_ClockPhaseFirstEdge;

    master_config.direction = (spi_cfg->operation & SPI_TRANSFER_LSB)
        ? kLPSPI_LsbFirst
        : kLPSPI_MsbFirst;

    master_config.baudRate = spi_cfg->frequency;

    clock_dev = device_get_binding(config->clock_name);
    if (clock_dev == NULL) {
        return -EINVAL;
    }

    if (clock_control_get_rate(clock_dev, config->clock_subsys, &clock_freq)) {
        return -EINVAL;
    }

    LPSPI_MasterInit(base, &master_config, clock_freq);

    LPSPI_MasterTransferCreateHandle(base, &data->m_handle, spi_mcux_master_transfer_callback, dev);

    data->ctx.config = spi_cfg;
    spi_context_cs_configure(&data->ctx);

    return 0;
}

static int spi_mcux_configure(struct device *dev,
                  const struct spi_config *spi_cfg)
{
    const struct spi_mcux_config *config = dev->config->config_info;
    u16_t master;
    int status;

    master = SPI_OP_MODE_GET(spi_cfg->operation);

    if (master) {
        status = master_init(config->base, dev, spi_cfg);
    } else {
        status = slave_init(config->base, dev, spi_cfg);
    }

    return status;
}

static int transceive(struct device *dev,
              const struct spi_config *spi_cfg,
              const struct spi_buf_set *tx_bufs,
              const struct spi_buf_set *rx_bufs,
              bool asynchronous,
              struct k_poll_signal *signal)
{
    struct spi_mcux_data *data = dev->driver_data;
    int ret;

    spi_context_lock(&data->ctx, asynchronous, signal);

    ret = spi_mcux_configure(dev, spi_cfg);
    if (ret) {
        goto out;
    }

    spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

    spi_context_cs_control(&data->ctx, true);

    spi_mcux_transfer_next_packet(dev);

    ret = spi_context_wait_for_completion(&data->ctx);
out:
    spi_context_release(&data->ctx, ret);

    return ret;
}

static int spi_mcux_transceive(struct device *dev,
                   const struct spi_config *spi_cfg,
                   const struct spi_buf_set *tx_bufs,
                   const struct spi_buf_set *rx_bufs)
{
    return transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_mcux_transceive_async(struct device *dev,
                     const struct spi_config *spi_cfg,
                     const struct spi_buf_set *tx_bufs,
                     const struct spi_buf_set *rx_bufs,
                     struct k_poll_signal *async)
{
    return transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_mcux_release(struct device *dev,
              const struct spi_config *spi_cfg)
{
    struct spi_mcux_data *data = dev->driver_data;

    spi_context_unlock_unconditionally(&data->ctx);

    return 0;
}

static int spi_mcux_init(struct device *dev)
{
    const struct spi_mcux_config *config = dev->config->config_info;
    struct spi_mcux_data *data = dev->driver_data;

    config->irq_config_func(dev);

    spi_context_unlock_unconditionally(&data->ctx);

    return 0;
}

static const struct spi_driver_api spi_mcux_driver_api = {
    .transceive = spi_mcux_transceive,
#ifdef CONFIG_SPI_ASYNC
    .transceive_async = spi_mcux_transceive_async,
#endif
    .release = spi_mcux_release,
};

#ifdef CONFIG_SPI_MCUX_LPSPI_1
static void spi_mcux_config_func_1(struct device *dev);

static const struct spi_mcux_config spi_mcux_config_1 = {
    .base = (LPSPI_Type *) CONFIG_SPI_1_BASE_ADDRESS,
    .clock_name = CONFIG_SPI_1_CLOCK_NAME,
    .clock_subsys = (clock_control_subsys_t) CONFIG_SPI_1_CLOCK_SUBSYS,
    .irq_config_func = spi_mcux_config_func_1,
};

static struct spi_mcux_data spi_mcux_data_1 = {
    SPI_CONTEXT_INIT_LOCK(spi_mcux_data_1, ctx),
    SPI_CONTEXT_INIT_SYNC(spi_mcux_data_1, ctx),
};

DEVICE_AND_API_INIT(spi_mcux_1, CONFIG_SPI_1_NAME, &spi_mcux_init,
            &spi_mcux_data_1, &spi_mcux_config_1,
            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
            &spi_mcux_driver_api);

static void spi_mcux_config_func_1(struct device *dev)
{
    IRQ_CONNECT(CONFIG_SPI_1_IRQ, CONFIG_SPI_1_IRQ_PRI,
            spi_mcux_isr, DEVICE_GET(spi_mcux_1), 0);

    irq_enable(CONFIG_SPI_1_IRQ);
}
#endif /* CONFIG_SPI_MCUX_LPSPI_1 */
