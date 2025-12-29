defmodule BbMujocoWeb.MjcfController do
  @moduledoc """
  Controller for serving MJCF robot models and mesh assets to the browser.

  The browser's MuJoCo WASM needs the MJCF XML and any associated
  mesh assets to load the robot model.

  ## Asset Directory

  Mesh files (STL, OBJ) should be placed in `priv/assets/`.
  The MJCF exporter references meshes by basename, and this controller
  serves them from the shared assets directory.
  """

  use BbMujocoWeb, :controller

  @assets_dir "priv/assets"

  @doc """
  Serve the MJCF XML for a robot.

  GET /api/mujoco/mjcf/:robot
  """
  def show(conn, %{"robot" => robot_name}) do
    case get_robot_module(robot_name) do
      {:ok, robot_module} ->
        mjcf = BB.Mujoco.Exporter.to_mjcf(robot_module)

        conn
        |> put_resp_content_type("application/xml")
        |> send_resp(200, mjcf)

      {:error, :not_found} ->
        conn
        |> put_status(:not_found)
        |> json(%{error: "Robot not found: #{robot_name}"})
    end
  end

  @doc """
  List available mesh assets.

  GET /api/mujoco/mjcf/:robot/assets

  Returns all assets in priv/assets/ (shared across all robots).
  """
  def list_assets(conn, %{"robot" => _robot_name}) do
    assets_path = assets_dir()

    if File.dir?(assets_path) do
      files =
        list_files_recursive(assets_path, "")
        |> Enum.filter(&valid_asset_path?/1)
        |> Enum.sort()

      json(conn, %{assets: files, count: length(files)})
    else
      json(conn, %{assets: [], count: 0})
    end
  end

  @doc """
  Serve a specific mesh asset.

  GET /api/mujoco/mjcf/:robot/assets/*path

  Serves files from priv/assets/.
  Supports subdirectories like meshes/robot_arm/link1.stl
  """
  def show_asset(conn, %{"robot" => _robot_name, "path" => path_parts}) do
    relative_path = Path.join(path_parts)

    if valid_asset_path?(relative_path) do
      full_path = Path.join(assets_dir(), relative_path)

      if File.exists?(full_path) do
        content = File.read!(full_path)
        content_type = asset_content_type(relative_path)

        conn
        |> put_resp_content_type(content_type)
        |> send_resp(200, content)
      else
        conn
        |> put_status(:not_found)
        |> json(%{error: "Asset not found: #{relative_path}"})
      end
    else
      conn
      |> put_status(:bad_request)
      |> json(%{error: "Invalid asset path"})
    end
  end

  # ===========================================================================
  # Private Functions
  # ===========================================================================

  defp assets_dir do
    Application.app_dir(:bb_mujoco, @assets_dir)
  end

  defp get_robot_module(robot_name) do
    module = String.to_atom("Elixir.#{robot_name}")

    case Code.ensure_compiled(module) do
      {:module, _} ->
        if function_exported?(module, :robot, 0) do
          {:ok, module}
        else
          {:error, :not_found}
        end

      {:error, _} ->
        {:error, :not_found}
    end
  end

  # Recursively list all files, returning relative paths
  defp list_files_recursive(dir, prefix) do
    full_path = if prefix == "", do: dir, else: Path.join(dir, prefix)

    case File.ls(full_path) do
      {:ok, entries} ->
        Enum.flat_map(entries, fn entry ->
          entry_path = if prefix == "", do: entry, else: Path.join(prefix, entry)
          full_entry_path = Path.join(full_path, entry)

          if File.dir?(full_entry_path) do
            list_files_recursive(dir, entry_path)
          else
            [entry_path]
          end
        end)

      {:error, _} ->
        []
    end
  end

  defp valid_asset_path?(path) do
    # Allow common mesh formats with subdirectory paths
    valid_extensions = ~w(.stl .obj .dae .png .jpg .jpeg)

    ext = Path.extname(path) |> String.downcase()

    # Security: no path traversal, no backslashes
    ext in valid_extensions and
      not String.contains?(path, "..") and
      not String.contains?(path, "\\")
  end

  defp asset_content_type(filename) do
    case Path.extname(filename) |> String.downcase() do
      ".stl" -> "application/octet-stream"
      ".obj" -> "text/plain"
      ".dae" -> "application/xml"
      ".png" -> "image/png"
      ".jpg" -> "image/jpeg"
      ".jpeg" -> "image/jpeg"
      _ -> "application/octet-stream"
    end
  end
end
