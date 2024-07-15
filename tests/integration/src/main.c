
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/shell/shell.h>

#include <argos_smd.h>


const struct device *dev = DEVICE_DT_GET_ONE(arribada_argossmd);

int main(void)
{
	return 0;
}

static int cmd_test_version(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

    if(argos_read_firmware_version(dev)) {
        shell_print(sh, "Error getting version");
        return ENODATA;
    } else {	
        struct argos_smd_data *data = (struct argos_smd_data *)dev->data;
        uint8_t *msg = data->response.data;
        shell_print(sh, "%s", msg);
        return 0;
    }
}

static int cmd_test_ping(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "pong");

	return 0;
}
static int cmd_test_smd(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
	ARG_UNUSED(argv);

    if(argos_read_ping(dev)) {
        shell_print(sh, "Error getting version");
        return ENODATA;
    } else {	
        struct argos_smd_data *data = (struct argos_smd_data *)dev->data;
        uint8_t *msg = data->response.data;
        shell_print(sh, "%s", msg);
        return 0;
    
}
static int cmd_test_params(const struct shell *sh, size_t argc,
                           char **argv)
{
        int cnt;

        shell_print(sh, "argc = %d", argc);
        for (cnt = 0; cnt < argc; cnt++) {
                shell_print(sh, "  argv[%d] = %s", cnt, argv[cnt]);
        }
        return 0;
}

/* Creating subcommands (level 1 command) array for command "test". */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_test,
        SHELL_CMD(params, NULL, "Print params command.",
                                               cmd_test_params),
        SHELL_CMD(ping,   NULL, "Ping command.", cmd_test_ping),
        SHELL_CMD(version,   NULL, "Version command.", cmd_test_version),

        SHELL_SUBCMD_SET_END
);
/* Creating root (level 0) command "test" without a handler */
SHELL_CMD_REGISTER(test, &sub_test, "Test commands", NULL);