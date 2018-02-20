        AREA    altera_conf_section, DATA, READONLY
        EXPORT  altera_conf_image

altera_conf_image
        INCBIN  altconf.rbf
altera_conf_image_end


altera_conf_image_length
        DCD     altera_conf_image_end - altera_conf_image

        EXPORT  altera_conf_image_length

        END