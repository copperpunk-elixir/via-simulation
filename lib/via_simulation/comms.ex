defmodule ViaSimulation.Comms do
  require Logger
  require ViaUtils.Shared.Groups, as: Groups
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaTelemetry.Ubx.StandardNav.Pvt, as: Pvt
  require ViaTelemetry.Ubx.StandardNav.Relposned, as: Relposned
  require ViaTelemetry.Ubx.Companion.DtAccelGyro, as: DtAccelGyro

  @spec publish_gps_itow_position_velocity(any(), map(), map(), any()) :: atom()
  def publish_gps_itow_position_velocity(operator_name, position_rrm, velocity_mps, group) do
    if group == Groups.virtual_uart_gps() do
      %{
        SVN.latitude_rad() => lat_rad,
        SVN.longitude_rad() => lon_rad,
        SVN.altitude_m() => height_m
      } = position_rrm

      %{
        SVN.v_north_mps() => v_north_mps,
        SVN.v_east_mps() => v_east_mps,
        SVN.v_down_mps() => v_down_mps
      } = velocity_mps

      value_map = %{
        Pvt.iTOW() => 1,
        Pvt.lat() => ViaUtils.Math.rad2deg(lat_rad),
        Pvt.lon() => ViaUtils.Math.rad2deg(lon_rad),
        Pvt.height() => height_m * 1000,
        Pvt.velN() => v_north_mps * 1000,
        Pvt.velE() => v_east_mps * 1000,
        Pvt.velD() => v_down_mps * 1000,
        Pvt.fixType() => 3
      }

      # Logger.debug("values: #{inspect(msg_values)}")

      msg =
        UbxInterpreter.construct_message_from_map(
          Pvt.class(),
          Pvt.id(),
          Pvt.bytes(),
          Pvt.multipliers(),
          Pvt.keys(),
          value_map
        )

      # Logger.debug("GPS Nav msg: #{msg}")

      ViaUtils.Comms.send_global_msg_to_group(
        operator_name,
        {:circuits_uart, 0, msg},
        self(),
        group
      )
    else
      itow_s = nil

      values = %{
        SVN.itow_s() => itow_s,
        SVN.position_rrm() => position_rrm,
        SVN.velocity_mps() => velocity_mps
      }

      ViaUtils.Comms.cast_global_msg_to_group(
        operator_name,
        {group, values},
        self()
      )
    end
  end

  @spec publish_gps_relheading(any(), float(), any()) :: atom()
  def publish_gps_relheading(operator_name, rel_heading_rad, group) do
    # TODO: Get correct value for itow_ms
    if group == Groups.virtual_uart_gps() do
      value_map = %{
        Relposned.relPosHeading() => ViaUtils.Math.rad2deg(rel_heading_rad),
        Relposned.relPosLength() => 100,
        Relposned.flags() => 261
      }

      msg =
        UbxInterpreter.construct_message_from_map(
          Relposned.class(),
          Relposned.id(),
          Relposned.bytes(),
          Relposned.multipliers(),
          Relposned.keys(),
          value_map
        )

      # Logger.debug("GPS hdg msg: #{msg}")

      ViaUtils.Comms.send_global_msg_to_group(
        operator_name,
        {:circuits_uart, 0, msg},
        self(),
        group
      )

      # Logger.debug("pub relhdg: #{ViaUtils.Format.eftb_deg(rel_heading_rad, 1)}")
    else
      itow_s = nil

      values = %{SVN.itow_s() => itow_s, SVN.yaw_rad() => rel_heading_rad}

      ViaUtils.Comms.cast_global_msg_to_group(
        operator_name,
        {group, values},
        self()
      )
    end
  end

  @spec publish_dt_accel_gyro(any(), float(), map(), map(), any()) :: atom()
  def publish_dt_accel_gyro(operator_name, dt_s, accel_mpss, gyro_rps, group) do
    value_map =
      %{SVN.dt_s() => dt_s}
      |> Map.merge(accel_mpss)
      |> Map.merge(gyro_rps)

    # Logger.debug("pub dtaccgy: #{ViaUtils.Format.eftb_map(values,4)}")

    if group == Groups.virtual_uart_dt_accel_gyro() do
      msg =
        UbxInterpreter.construct_message_from_map(
          DtAccelGyro.class(),
          DtAccelGyro.id(),
          DtAccelGyro.bytes(),
          DtAccelGyro.multipliers(),
          DtAccelGyro.keys(),
          value_map
        )

      ViaUtils.Comms.send_global_msg_to_group(
        operator_name,
        {:circuits_uart, 0, msg},
        self(),
        group
      )

      # Logger.debug("pub dtaccgy: #{ViaUtils.Format.eftb_map(values, 4)}")
      # Logger.debug("pub group: #{inspect(group)}")
    else
      ViaUtils.Comms.cast_global_msg_to_group(
        operator_name,
        {group, value_map},
        self()
      )
    end
  end

  @spec publish_airspeed(any(), float(), any()) :: atom()
  def publish_airspeed(operator_name, airspeed_mps, group) do
    # Logger.debug("pub A/S: #{ViaUtils.Format.eftb(airspeed_mps, 1)}")

    ViaUtils.Comms.send_global_msg_to_group(
      operator_name,
      {group, airspeed_mps},
      self()
    )
  end

  @spec publish_downward_range_distance(any(), number(), module, any()) :: atom()
  def publish_downward_range_distance(operator_name, range_m, downward_range_module, group) do
    # Logger.debug("pub tof: #{ViaUtils.Format.eftb(range_meas, 1)}")

    if group == Groups.virtual_uart_downward_range() do
      msg = apply(downward_range_module, :create_message_for_range_m, [range_m])
      # Logger.debug("DR msg: #{msg}")

      ViaUtils.Comms.send_global_msg_to_group(
        operator_name,
        {:circuits_uart, 0, msg},
        self(),
        group
      )
    else
      ViaUtils.Comms.cast_global_msg_to_group(
        operator_name,
        {group, range_m},
        self()
      )
    end
  end
end
